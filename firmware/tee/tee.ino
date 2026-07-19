#include <V2Base.h>
#include <V2Device.h>
#include <V2LED.h>
#include <V2Link.h>
#include <V2MIDI.h>

V2DEVICE_METADATA("com.versioduo.tee", 3, "versioduo:samd:tee");

namespace {
  V2LED::WS2812 LED(4, PIN_LED_WS2812, &sercom2, SPI_PAD_0_SCK_1, PIO_SERCOM);
  V2Link::Port  Plug(&SerialPlug, PIN_SERIAL_PLUG_TX_ENABLE);
  V2Link::Port  Socket(&SerialSocket, PIN_SERIAL_SOCKET_TX_ENABLE);
  V2Link::Port  SocketNode(&SerialSocketNode, PIN_SERIAL_SOCKET_NODE_TX_ENABLE);

  struct Setup {
    enum LEDs : uint8_t { Local, Plug, SocketNode, Socket };
  };

  class Device : public V2Device {
  public:
    uint8_t passthrough{};

    Device() : V2Device() {
      metadata.vendor      = "Versio Duo";
      metadata.product     = "V2 tee";
      metadata.description = "V2 Link Node";
      metadata.home        = "https://versioduo.com/#tee";

      system.download  = "https://versioduo.com/download";
      system.configure = "https://versioduo.com/configure";

      usb.ports.standard = 8;
    }

    auto light(Setup::LEDs led, const V2MIDI::Packet& midi) {
      switch (midi.type()) {
        case V2MIDI::Packet::Status::NoteOff:
          LED.setBrightness(led, 0);
          break;

        case V2MIDI::Packet::Status::NoteOn:
          LED.setHSV(led, V2Colour::Orange, 0.95, 0.5);
          break;

        case V2MIDI::Packet::Status::ControlChange:
          LED.setHSV(led, V2Colour::Cyan, 0.95, 0.5);
          break;
      }
    }

  private:
    enum class CC {
      Node = V2MIDI::CC::Controller9,
    };

    auto handleReset() -> void override {
      passthrough = 0;
      LED.reset();
    }

    auto handleSystemReset() -> void override {
      reset();
    }

    auto handleSend(V2MIDI::Packet* midi) -> bool override {
      usb.midi.send(*midi);
      Plug.send(*midi);
      return true;
    }

    auto handleControlChange(uint8_t channel, uint8_t controller, uint8_t value) -> void override {
      if (channel > 0)
        return;

      switch (controller) {
        case uint8_t(CC::Node):
          if (value < 0 || value > usb.ports.current - 1)
            break;

          passthrough = value;
          break;
      }
    }

    auto exportInput(JsonObject json) -> void override {
      auto jsonControllers{json["controllers"].to<JsonArray>()};
      {
        auto jsonController{jsonControllers.add<JsonObject>()};
        jsonController["name"]   = "Passthrough";
        jsonController["number"] = uint8_t(CC::Node);
        jsonController["max"]    = usb.ports.current - 1;
        jsonController["value"]  = passthrough;
      }
    }

    auto exportSystem(JsonObject json) -> void override {
      auto node{json["midi"]["passthrough"].to<JsonObject>()};
      node["controller"] = uint8_t(CC::Node);
      node["node"]       = passthrough;
    }
  } Device;

  // Dispatch USB MIDI packets.
  class MIDI {
  public:
    auto loop() {
      if (!Device.usb.midi.receive(_midi))
        return;

      if (_midi.port == 0) {
        // Addressed to this device.

        if (Device.passthrough == 0) {
          // Without passthrough, channel #1 addresses this device, the higher channels are forwarded to the node children.
          switch (_midi.type()) {
            case V2MIDI::Packet::Status::NoteOff:
            case V2MIDI::Packet::Status::NoteOn:
            case V2MIDI::Packet::Status::Aftertouch:
            case V2MIDI::Packet::Status::ControlChange:
            case V2MIDI::Packet::Status::ProgramChange:
            case V2MIDI::Packet::Status::AftertouchChannel:
            case V2MIDI::Packet::Status::PitchBend:
              if (_midi.channel() > 0) {
                auto address{_midi.channel() - 1};
                _midi.channel(0);
                SocketNode.send(V2Link::Packet(address, _midi));
                Device.light(Setup::LEDs::SocketNode, _midi);
                break;
              }
              [[fallthrough]];

            default:
              Device.dispatch(&Device.usb.midi, &_midi);
              Device.light(Setup::LEDs::Local, _midi);
          }

          return;
        }

        // With passthrough, all messages are forwarded to the specified node child.
        SocketNode.send(V2Link::Packet(Device.passthrough - 1, _midi));
        Device.light(Setup::LEDs::SocketNode, _midi);

        // MIDI Reset messages are intercepted and reset this device too. This ensures
        // the user interface can always revert back to talking to this device.
        if (_midi.type() == V2MIDI::Packet::Status::SystemReset)
          Device.dispatch(&Device.usb.midi, &_midi);

        return;
      }

      auto address{_midi.port - 1};

      // Addressed to the direct children devices.
      if (Device.passthrough == 0) {
        // The channel of the incoming message is patched into the port of the outgoing
        // message. System messages are addressed to the node itself.
        //
        // The receiving device will use the port number to route the packet to its node children.

        switch (_midi.type()) {
          case V2MIDI::Packet::Status::NoteOff:
          case V2MIDI::Packet::Status::NoteOn:
          case V2MIDI::Packet::Status::Aftertouch:
          case V2MIDI::Packet::Status::ControlChange:
          case V2MIDI::Packet::Status::ProgramChange:
          case V2MIDI::Packet::Status::AftertouchChannel:
          case V2MIDI::Packet::Status::PitchBend:
            _midi.port = _midi.channel();
            _midi.channel(0);
            break;

          default:
            _midi.port = 0;
        }

      } else {
        // All messages are addressed to the specified node child.
        _midi.port = Device.passthrough;
      }

      V2Link::Packet p(address, _midi);
      Socket.send(p);
      Device.light(Setup::LEDs::Socket, p.midi);
    }

  private:
    V2MIDI::Packet _midi;
  } MIDI;

  // Dispatch Link packets.
  class Link : public V2Link {
  public:
    Link() : V2Link(&Plug, &Socket, &SocketNode) {
      Device.link = this;
    }

  private:
    // Receive a host event from our parent device.
    auto receivePlug(V2Link::Packet& p) -> void override {
      if (p.type == V2Link::Packet::Type::MIDI) {
        if (p.midi.port == 0) {
          Device.dispatch(&Plug, &p.midi);
          Device.light(Setup::LEDs::Plug, p.midi);
          return;
        }

        auto address{p.midi.port - 1};
        p.midi.port = 0;
        SocketNode.send(V2Link::Packet(address, p.midi));
        Device.light(Setup::LEDs::SocketNode, p.midi);
      }
    }

    // Forward children device events to the host.
    auto receiveSocket(V2Link::Packet& p) -> void override {
      if (p.type == V2Link::Packet::Type::MIDI) {
        if (Device.usb.midi.connected()) {
          if (p.midi.port > 0) {
            switch (p.midi.type()) {
              case V2MIDI::Packet::Status::NoteOff:
              case V2MIDI::Packet::Status::NoteOn:
              case V2MIDI::Packet::Status::Aftertouch:
              case V2MIDI::Packet::Status::ControlChange:
              case V2MIDI::Packet::Status::ProgramChange:
              case V2MIDI::Packet::Status::AftertouchChannel:
              case V2MIDI::Packet::Status::PitchBend:
                p.midi.channel(p.midi.port);
                break;
            }
          }

          p.midi.port = p.address;
          Device.usb.midi.send(p.midi);
          Device.light(Setup::LEDs::Socket, p.midi);
        }
      }
    }

    auto receiveSocketNode(V2Link::Packet& p) -> void override {
      if (p.type == V2Link::Packet::Type::MIDI) {
        p.midi.port = p.address + 1;
        Plug.send(p.midi);
        Device.light(Setup::LEDs::Plug, p.midi);

        switch (p.midi.type()) {
          case V2MIDI::Packet::Status::NoteOff:
          case V2MIDI::Packet::Status::NoteOn:
          case V2MIDI::Packet::Status::Aftertouch:
          case V2MIDI::Packet::Status::ControlChange:
          case V2MIDI::Packet::Status::ProgramChange:
          case V2MIDI::Packet::Status::AftertouchChannel:
          case V2MIDI::Packet::Status::PitchBend:
            p.midi.channel(p.address + 1);
            break;

          default:
            if (p.address + 1 != Device.passthrough)
              break;
        }

        p.midi.port = 0;
        Device.usb.midi.send(p.midi);
      }
    }
  } Link;
}

auto setup() -> void {
  Serial.begin(9600);
  LED.begin();
  LED.setMaxBrightness(0.5);

  // Set the SERCOM interrupt priority, it requires a stable ~300 kHz interrupt
  // frequency. The calls need to be after begin().
  Link.begin();
  setSerialPriority(&SerialPlug, 2);
  setSerialPriority(&SerialSocket, 1);
  setSerialPriority(&SerialSocketNode, 2);

  Device.begin();
  Device.reset();
}

auto loop() -> void {
  LED.loop();
  MIDI.loop();
  Link.loop();
  Device.loop();

  if (Link.idle() && Device.idle())
    Device.sleep();
}
