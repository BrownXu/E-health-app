# -*- coding: utf-8 -*-
"""
Created on Mon Oct 21 09:53:31 2019

@author: Brown
"""

import pygatt
from time import sleep
import numpy as np
import threading
import urllib.request
import pywt

url = 'http://10.16.202.74:8000/ehealth/?msg_content='


adapter = pygatt.backends.GATTToolBackend()
adapter.start()

emg = np.zeros(500,dtype = int)
temp = 0
plu = 999
spo = 999 
emgc = 0

emgerr= np.ones(500,dtype = int)
temperr = 0
pluerr = 999
spoerr = 999
scount = 0
sendname = 0


try:
    device = adapter.connect('00:07:80:DE:DB:E3')
    def getemg():
            global emg
            global emgc
            while True:
                emgbit= device.char_read('3b54d144-a5f4-4444-aabc-7ada95be9498')
                if emgc <500:
                    emg[emgc]= int.from_bytes(emgbit,byteorder='little')
                    emgc = emgc +1
                    sleep(0.01)
                if emgc == 500:
                    emg = np.roll(emg,-1)
                    emg[emgc-1]= int.from_bytes(emgbit,byteorder='little')
                    sleep(0.01)
                am = np.mean(emg,axis= 0)
                emg = emg - am
                
    def gettemp ():
        global temp
        while True:
            temp = int.from_bytes(device.char_read('362ba79d-b620-41d3-89ee-48f865559129'),byteorder= 'little')/100
            sleep(20)
        
    def getspo2 ():
        while True:
            global plu
            global spo
            spobit = device.char_read('362ba7aa-b620-41d3-89ee-48f865559129')
            plu = int.from_bytes(spobit[0:2],byterorder = 'little')
            spo = int.from_bytes(spobit[2:4],byterorder = 'little')
            sleep(40)

    def emgfft(emgsignal):
        fftsignal = np.fft.rfft(emgsignal)
        fftsignal = np.real(fftsignal[1:251]).astype('float32')# remove the DC part
        return fftsignal
    
    def emgdenoise(emgraw):
        w = pywt.Wavelet('db8') 
        maxlev = pywt.dwt_max_level(len(emgraw), w.dec_len)
        threshold = 0.06
        coeffs = pywt.wavedec(emgraw, 'db8', level=maxlev)
        for i in range(1, len(coeffs)):
            coeffs[i] = pywt.threshold(coeffs[i], threshold*max(coeffs[i]))
        dataout = pywt.waverec(coeffs, 'db8').astype('float32')
        return dataout
    
    def mediafreq(fftemg):
        m = fftemg.__add__(abs (fftemg.min()))
        t = np.sum(m) /2 #half product
        k = 0  #sum
        s = [] # sub
        for i in range (len(m)):
            k = k+ m[i] # current sum
            s.append(abs(t-k))
        med = s.index(min(s))
        return med
    
    def urlget():
        global scount
        global url
        global emg
        global plu
        global spo
        global temp
        while True:
            if scount == 20:
                sendemg = emgdenoise(emg)
                ffemgs = emgfft(sendemg)
                data = {'emg':sendemg,'femg':ffemgs,'med':mediafreq(ffemgs),'temp':temp}
                scount = scount +1
            elif scount == 40:
                sendemg = emgdenoise(emg)
                data = {'emg':sendemg,'femg':ffemgs,'med':mediafreq(ffemgs),'plus':plu,'spo':spo}
                scount = 0
            else:
                sendemg = emgdenoise(emg)
                ffemgs = emgfft(sendemg)
                data = {'emg':sendemg,'femg':ffemgs,'med':mediafreq(ffemgs)}
                scount = scount +1
            urllib.request.urlopen(url+data)
            sleep(1)

    
    t1 = threading.Thread(target=getemg)
    t2 = threading.Thread(target=gettemp)
    t3 = threading.Thread(target=getspo2)
    t4 = threading.Thread(target=urlget)

    sleep(0.5)
except:
    try:
        adapter.stop()
        sleep(1)
        adapter.start()
        device = adapter.connect('00:07:80:DE:DB:E3')
        sleep(0.5)
    except:
        print("please check Bluetooth connection")
    else:
        print("Try to restart Bluetooth again")
    finally:
        emg = emgerr
        temp = temperr
        plu = pluerr
        spo = spoerr
        t1.start()
        sleep(0.5)
        t2.start()
        sleep(0.5)
        t3.start()
        sleep(0.5)
        t4.start()
        
        
else:
    device.subscribe('ade7a273-89f9-49e1-b9d4-3cb36bce261b') ##ecg,to activate sending status
    sleep(5)
    try:
        emgrea = device.char_read('3b54d144-a5f4-4444-aabc-7ada95be9498')
    except:
        sleep(2)
        print("Try to restart your device")
    else:
        t1.start()
        sleep(0.3)
        t2.start()
        sleep(0.4)
        t3.start()
        sleep(0.5)
        t4.start()
        
        
        
