import socket

def Main():
   
    #host = '172.26.166.129' #Server ip 1082
    host = '172.26.163.219' #Server ip 1075
    port = 4000

    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.bind((host, port))

    print("Server Started")
    while True:
        data, addr = s.recvfrom(1024)
        data = data.decode('utf-8')
        print("Message from: " + str(addr))
        print("From connected user: " + data)
        print("Sending: " + data)
        s.sendto(data.encode('utf-8'), addr)
    c.close()

if __name__=='__main__':
    Main()
