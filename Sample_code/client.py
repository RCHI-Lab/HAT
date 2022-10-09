import socket

def Main():

host='172.26.98.229' #client ip
port = 4005

server = ('172.26.166.129', 4000)

s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
s.bind((host,port))

message = input("-> ")
while message !='q':
    s.sendto(message.encode('utf-8'), server)
    data, addr = s.recvfrom(1024)
    data = data.decode('utf-8')
    print("Received from server: " + data)
    message = input("-> ")
s.close()


if name=='main':
Main()

