#!/usr/bin/env python
import time
from ur_driver.io_interface import *

if __name__ == "__main__":
    print "testing io-interface"
    get_states()
    print "listener has been activated"
    set_states()
    print "service-server has been started"
    #~ i=0
    #~ while(i<10):
        #~ set_tool_voltage(12)
        #~ set_digital_out(0, True)
        #~ set_analog_out(0, 0.75)
        #~ #print "Flags are currently not supported"
        #~ ##set_flag(0, True)
        #~ ##print(Flag_States[0])
        #~ print(Analog_Out_States[0])
        #~ print(Digital_Out_States[0])
        #~ time.sleep(1)
        #~ set_tool_voltage(24)
        #~ set_digital_out(0, False)
        #~ set_analog_out(0, 0.25)
        #~ #print "Flags are currently not supported"
        #~ ##set_flag(0, False)
        #~ ##print(Flag_States[0])
        #~ print(Analog_Out_States[0])
        #~ print(Digital_Out_States[0])
        #~ time.sleep(1)
        #~ print("i:"+str(i))
        #~ i+=1
    time.sleep(2)
    set_io_val(0, 0.005)
    print("Cerrar -> "+ str(Digital_Out_States[16]))
    time.sleep(3)
    set_digital_out(16, False)
    print("Cerrar -> "+ str(Digital_Out_States[16]))
    time.sleep(3)
    set_digital_out(16, True)
    print("Cerrar -> "+ str(Digital_Out_States[16]))
    time.sleep(3)
    set_digital_out(16, False)
    print("Cerrar -> "+ str(Digital_Out_States[16]))
    
    time.sleep(2) 
    print("Analog")
    print(str(Analog_Out_States[0]) + " , " + str(Analog_Out_States[1]))
    

    print("Analog_In")
    print(str(Analog_In_States[0]) + " , " + str(Analog_In_States[1]))
    
    print("Digital")
    print(str(Digital_Out_States[0]) + " , " + str(Digital_Out_States[1]) 
            + " , " + str(Digital_Out_States[2]) + " , " + str(Digital_Out_States[3])
            + " , " + str(Digital_Out_States[4]) + " , " + str(Digital_Out_States[5])
            + " , " + str(Digital_Out_States[6]) + " , " + str(Digital_Out_States[7])
            + " , " + str(Digital_Out_States[8]) + " , " + str(Digital_Out_States[9])
            + " , " + str(Digital_Out_States[10]) + " , " + str(Digital_Out_States[11])
            + " , " + str(Digital_Out_States[12]) + " , " + str(Digital_Out_States[13])
            + " , " + str(Digital_Out_States[14]) + " , " + str(Digital_Out_States[15])
            + " , " + str(Digital_Out_States[16]) + " , " + str(Digital_Out_States[17]))           
    print("Digital In")
    print(str(Digital_In_States[0]) + " , " + str(Digital_In_States[1]) 
            + " , " + str(Digital_In_States[2]) + " , " + str(Digital_In_States[3])
            + " , " + str(Digital_In_States[4]) + " , " + str(Digital_In_States[5])
            + " , " + str(Digital_In_States[6]) + " , " + str(Digital_In_States[7])
            + " , " + str(Digital_In_States[8]) + " , " + str(Digital_In_States[9])
            + " , " + str(Digital_In_States[10]) + " , " + str(Digital_In_States[11])
            + " , " + str(Digital_In_States[12]) + " , " + str(Digital_In_States[13])
            + " , " + str(Digital_In_States[14]) + " , " + str(Digital_In_States[15])
            + " , " + str(Digital_In_States[16]) + " , " + str(Digital_In_States[17])) 

    time.sleep(1)
