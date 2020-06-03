import os
import random
import time
import datetime
from socket import *
from multiprocessing import Pool

HOST = '192.168.43.178'
POST = [21569]

limit = datetime.timedelta(minutes = 30)

def subtime(time1,time2):
    time1=datetime.datetime.strptime(time1,"%Y-%m-%d %H:%M:%S")
    time2=datetime.datetime.strptime(time2,"%Y-%m-%d %H:%M:%S")
    return time2-time1

def connect(ADDR):
    global reset_time
    while True:
        try:
            client = socket(AF_INET, SOCK_STREAM)
            client.connect(ADDR)
            while True:
                data = client.recv(1024)
                print(ADDR, data.decode())   
                if data.decode()=='connect':
                    client.send('connectOK'.encode(encoding='utf-8'))
                    out_r = os.popen('./empty').read().replace('\n', '')
                    print(out_r)
                    if out_r == 'OK':
                        client.send('resetOK'.encode(encoding='utf-8'))
                        reset_time = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
                elif data.decode()=='detect':
                    client.send('detectOK'.encode(encoding='utf-8'))
                    while True:
                        out_o = os.popen('./detect').read().replace('\n', '')
                        out_o = out_o.replace('OK', '')
                        #if out2 != 'empty' and out2 != 'nonempty':
                        #    out2='default'
                        print(out_o)
                        client.send(out_o.encode(encoding='utf-8'))
                        # reset regularly (30 minutes)
                        now_time=datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
                        if out_o == 'empty' and subtime(reset_time,now_time) >= limit :
                            out_r1 = os.popen('./empty').read().replace('\n', '')
                            if out_r1 == 'OK':
                                reset_time = now_time
                if not data: break
        except:
            print('continue')
            continue

    client.settimeout(1)
    client.setsockopt(SOL_SOCKET,SO_REUSEADDR,1) 
    client.shutdown(2)
    client.close() 

if __name__ == '__main__':
    ADDR=[]
    ID = 0
    while ID < len(POST):
        addr=(HOST,POST[ID])
        ADDR.append(addr)
        ID+=1
    
    reset_time = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    with Pool(len(HOST)) as p:
        p.map(connect,ADDR)







