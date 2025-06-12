# testing

SOCKET_UDP
/*
socket() creates an endpoint for communication and returns a file descriptor
that refers to that endpoint. (domain,type,protocol)

The recvfrom() function shall receive a message from a connection-mode or
connectionless-mode socket. It is normally used with connectionless-mode sockets
because it permits the application to retrieve the source address of received
data.
(socket, *buffer, length, flags, address, address_length)

length = buffer size
flags = 0

address = pointer to the address of the sender, points to a sockaddr structure
in which the sending address is to be stored

address_len
    Specifies the length of the sockaddr structure pointed to by the address,
socklen_t *addrlen argument

You're telling recvfrom():

    “Here’s a pointer to a buffer (cli_addr) that’s this big — go ahead and fill
it, and update clilen with how much you actually used.”
*/

FLATBIRD_EXAMPLE
/*
RCLCPP_INFO(this->get_logger(), "Setting up flatbuffers...");
  // Create the flatbuffer builder
  flatbuffers::FlatBufferBuilder builder(1024);

  // Create a FlatBuffer string
  auto msg_str0 = builder.CreateString("Hello");

  // Build the message object
  auto message_offset = TestProtocol::Createmessage(builder, 1, 42, msg_str0);

  // Finish the buffer — this sets the root and finalizes it
  builder.Finish(message_offset);

  // Now builder.GetBufferPointer() points to the serialized binary
  uint8_t *buf = builder.GetBufferPointer();
  size_t size = builder.GetSize();

  builder.Clear(); // reset builder to build next message

  flatbuffers::Verifier verifier(buf, size);
  if (!TestProtocol::VerifymessageBuffer(verifier)) {
    // handle invalid buffer
    RCLCPP_ERROR(this->get_logger(), "Invalid buffer");
  }

  // Get root object pointer from buffer
  const TestProtocol::message *msg = TestProtocol::Getmessage(buf);

  // Access fields
  uint8_t id1 = msg->id();
  int32_t value2 = msg->value();
  std::string msg_str8 = msg->msg()->str();
  RCLCPP_INFO(this->get_logger(), "ID: %d", id1);
  RCLCPP_INFO(this->get_logger(), "Value: %d", value2);
  RCLCPP_INFO(this->get_logger(), "Message: %s", msg_str8.c_str());
*/
  
