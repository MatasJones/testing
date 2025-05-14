# testing

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