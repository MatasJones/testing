#ifndef CUSTOM_SER_H
#define CUSTOM_SER_H

#include "../../custom_msg/flatbuff/message_generated.h"
#include "flatbuffers/flatbuffers.h"

class custom_ser {
public:
  /**
   * @brief Takes in a message, an id and a value and generates a flatbuffer
   * which can then be send via UDP/TCP
   *
   * @param msg
   * @param id
   * @param value
   * @param buffer to reference during sending
   */
  static void ser_msg(std::string msg, uint8_t id, float value,
                      flatbuffers::FlatBufferBuilder *builder, uint8_t *buf,
                      uint32_t *size) {

    builder->Clear();

    auto msg_str = builder->CreateString(msg);
    auto message_offset =
        TestProtocol::Createmessage(*builder, id, value, msg_str);

    builder->Finish(message_offset);

    uint32_t payload_size = builder->GetSize();
    uint8_t *buf_ptr = builder->GetBufferPointer();
    uint32_t size_le = htole32(payload_size);

    memcpy(buf, &size_le, sizeof(uint32_t));          // Write size prefix
    memcpy(buf + sizeof(uint32_t), buf_ptr, size_le); // Write FlatBuffer

    *size = sizeof(uint32_t) + size_le; // Total message size
  }

  /**
   * @brief Takes in a buffer and deserializes it to get the message, id and
   * value
   *
   * @param buf to pass in as reference
   * @param msg
   * @param id
   * @param value
   */
  static bool deser_msg(const uint8_t *buf, std::string &msg, uint8_t &id,
                        float &value) {

    // Extract the message size from the buffer
    uint32_t size;
    memcpy(&size, buf, sizeof(uint32_t));
    size = le32toh(size);

    flatbuffers::Verifier verifier(buf + 4, size);
    if (!TestProtocol::VerifymessageBuffer(verifier)) {
      // handle invalid buffer
      return 0;
    }

    // Deserialize the message
    auto message = TestProtocol::Getmessage(buf + 4);
    msg = message->msg()->str();
    id = message->id();
    value = message->value();

    return 1;
  }
};

#endif
