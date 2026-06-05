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
    auto handleReset() -> void override {
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

    auto handlePacket(V2MIDI::Packet* midi) -> void override {
      switch (midi->type()) {
        case V2MIDI::Packet::Status::NoteOff:
        case V2MIDI::Packet::Status::NoteOn:
        case V2MIDI::Packet::Status::Aftertouch:
        case V2MIDI::Packet::Status::ControlChange:
        case V2MIDI::Packet::Status::ProgramChange:
        case V2MIDI::Packet::Status::AftertouchChannel:
        case V2MIDI::Packet::Status::PitchBend: {
          auto address{midi->channel()};
          midi->channel(0);
          SocketNode.send(V2Link::Packet(address, *midi));
          light(Setup::LEDs::SocketNode, *midi);
          break;
        }
      }
    }
  } Device;

  // Dispatch MIDI packets.
  class MIDI {
  public:
    auto loop() {
      if (!Device.usb.midi.receive(_midi))
        return;

      if (_midi.port == 0) {
        Device.dispatch(&Device.usb.midi, &_midi);
        Device.light(Setup::LEDs::Local, _midi);

      } else {
        V2Link::Packet p(_midi.port - 1, _midi);
        p.midi.port = 0;
        Socket.send(p);
        Device.light(Setup::LEDs::Socket, p.midi);
      }
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
        Device.dispatch(&Plug, &p.midi);
        Device.light(Setup::LEDs::Plug, p.midi);
      }
    }

    // Forward children device events to the host.
    auto receiveSocket(V2Link::Packet& p) -> void override {
      if (p.type == V2Link::Packet::Type::MIDI) {
        if (Device.usb.midi.connected()) {
          Device.light(Setup::LEDs::Socket, p.midi);
          p.midi.port = p.address;
          Device.usb.midi.send(p.midi);
        }
      }
    }

    auto receiveSocketNode(V2Link::Packet& p) -> void override {
      if (p.type != V2Link::Packet::Type::MIDI)
        return;

      switch (p.midi.type()) {
        case V2MIDI::Packet::Status::NoteOff:
        case V2MIDI::Packet::Status::NoteOn:
        case V2MIDI::Packet::Status::Aftertouch:
        case V2MIDI::Packet::Status::ControlChange:
        case V2MIDI::Packet::Status::ProgramChange:
        case V2MIDI::Packet::Status::AftertouchChannel:
        case V2MIDI::Packet::Status::PitchBend:
          p.midi.channel(p.address);
          Plug.send(p.midi);
          Device.usb.midi.send(p.midi);
          Device.light(Setup::LEDs::Plug, p.midi);
          break;
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
