
from operator import index
from tabnanny import check
import serial
import time

from socket import timeout

from tkinter import *
import queue

import threading
import serial.tools.list_ports


set_ind = 0
set_val = 0
servoVals = [0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0]
SelecServoVals = []
prevAngle = 0
j = 0
serv_comb = 0
loop_ite = 0
tf = False
send_rate = 12
end_x= 0
sum_ind = 0
myports = [tuple(p) for p in list(serial.tools.list_ports.comports())]
arduino_port = []
conn_bool = False
k = 0

def connectBoard():
    global arduinoData
    global conn_bool
    global myports
    global arduino_port
    try:
        arduinoData = serial.Serial("com7", baudrate = 250000, timeout =0.05)
    except:
        print("CONNECTION TO ARDUINO BOARD FAILED")
    else: 
        conn_bool = True
        myports = [tuple(p) for p in list(serial.tools.list_ports.comports())]
        print (myports)
        arduino_port = [port for port in myports if 'COM7' in port ][0]
        port_controller = threading.Thread(target=check_presence, args=(arduino_port, 0.1,))
        port_controller.setDaemon(True)
        port_controller.start()

def check_presence(correct_port, interval=0.1):
    global conn_bool
    global glob_status
    while conn_bool == True :
        myports = [tuple(p) for p in list(serial.tools.list_ports.comports())]
        if arduino_port not in myports:
            print ("Arduino has been disconnected!")
            glob_status.config(text="Arduino has been disconnected!",fg="red")
            conn_bool = False
            break
        time.sleep(interval)



class GUI:
    def __init__(self, master):
        
        connectBoard()
        self.master = master
        self.frame1 = LabelFrame(self.master, text='Servo Angle (in Degrees)', padx=5, pady=52, bg='black', fg='white')
        self.frame2 = LabelFrame(self.master,text='Controls',  padx=1, pady=1)
        self.sliders = LabelFrame(self.frame2,text='Sliders')
        self.test_servo = LabelFrame(root,text='Test Servo',padx=1, pady=10)
        global arduinoData 
        global conn_bool


        self.frame1.grid(row=0, column= 0, columnspan=1, sticky=N+S+W+E)
        self.frame2.grid(row=0, column= 1, columnspan=2, sticky=W+E)
        self.sliders.grid(row=1, column= 1, columnspan=3, pady=10)
        self.test_servo.grid(row=2, column= 0, columnspan=3,  sticky=W+E)
        
        self.min_grad = 0
        self.max_grad = 190
        self.selec = 0
        #TO DO: When Changing Combination restore the Angle vals
        self.s1_grads = [0]
        self.s2_grads = [0]
        self.s3_grads = [0]
        self.s4_grads = [0]
        ##
        self.j = 0
        self.i = 0
        #TODO: Simultanious Servo Control
        self.sim = 0
        ##

        ###GUI DEFINITIONS###
        root.title('Servo Controller v 1.0 ARDUINO')
        #LABELS
        self.butt_up = Button(self.frame2, text='Next. Combo', command=self.combUp)
        self.butt_down = Button(self.frame2, text= 'Prev. Combo', command=self.combDown)

        self.s1 = Label(self.frame1, text='Servo ' +  self.selecComb()[0]  + ': ' , bg='black', fg='white')
        self.s2 = Label(self.frame1, text='Servo ' +  self.selecComb()[1]  + ': ', bg='black', fg='white')
        self.s3 = Label(self.frame1, text='Servo ' +  self.selecComb()[2]  + ': ', bg='black', fg='white')
        self.s4 = Label(self.frame1, text='Servo ' +  self.selecComb()[3]  + ': ', bg='black', fg='white')

        self.val_s1 = Label(self.frame1, text= '0', bg='black', fg='yellow')
        self.val_s2 = Label(self.frame1, text= '0', bg='black', fg='yellow')
        self.val_s3 = Label(self.frame1, text= '0', bg='black', fg='yellow')
        self.val_s4 = Label(self.frame1, text= '0', bg='black', fg='yellow')

        #SCALE
        
        self.current_val1 = DoubleVar()
        self.servo1 = Scale(self.sliders, from_= self.min_grad, to=self.max_grad, command=self.getScale1,variable= self.current_val1, repeatdelay=500)
        self.current_val2 = DoubleVar()
        self.servo2 = Scale(self.sliders, from_= self.min_grad, to=self.max_grad, command=self.getScale2,variable= self.current_val2, repeatdelay=500)
        self.current_val3 = DoubleVar()
        self.servo3 = Scale(self.sliders, from_= self.min_grad, to=self.max_grad, command=self.getScale3,variable= self.current_val3, repeatdelay=500)
        self.current_val4 = DoubleVar()
        self.servo4 = Scale(self.sliders, from_= self.min_grad, to=self.max_grad, command=self.getScale4,variable= self.current_val4, repeatdelay=500)

        global glob_servo1 
        glob_servo1 = self.servo1
        global glob_servo2 
        glob_servo2 = self.servo2
        global glob_servo3 
        glob_servo3 = self.servo3
        global glob_servo4 
        glob_servo4 = self.servo4
        
        ##BUTTONS##
        self.test_button = Button(self.test_servo, command=self.conn)
        self.test_button.configure(
            text="Connect Arduino", background="Grey",
            padx=50
            )
        self.test_button.grid(row=0, column = 0, padx=2)

        ## GUI GRIDS ##

        #ENTRY

        self.butt_up.grid(row=2, column = 1, padx=10)
        self.butt_down.grid(row=2, column = 2, padx=10)

        #SCALE
        self.servo1.grid(row=0, column= 1)
        self.servo2.grid(row=0, column= 2)
        self.servo3.grid(row=0, column= 3)
        self.servo4.grid(row=0, column= 4)
        #LABELS
        self.s1.grid(row=0, column = 0)
        self.s2.grid(row=1, column = 0)
        self.s3.grid(row=2, column = 0)
        self.s4.grid(row=3, column = 0)

        self.val_s1.grid(row=0, column = 1)
        self.val_s2.grid(row=1, column = 1)
        self.val_s3.grid(row=2, column = 1)
        self.val_s4.grid(row=3, column = 1)
        #GUI STATUS BAR
        self.status = Label(
            root, bd = 1, relief=SUNKEN, anchor= E, text='Status')
        self.status.grid(row=4, column=0, columnspan=3, sticky=W+E)

        global glob_status
        glob_status = self.status

    ###FUNCTIONS###

    def combUp (self):
        global serv_comb
        comb_summ = serv_comb
        global SelecServoVals 
        SelecServoVals = []
        
        if serv_comb == 3:
            curr_text= 'Comb maxima seleccionada'
            self.status.config(text=curr_text)
            exit
        
        else:
            curr_text= 'Using Servo Combo ' + str(serv_comb)
            self.status.config(text=curr_text)
            comb_summ = comb_summ + 1
            exit
        
        serv_comb = comb_summ
        selec_ = self.selecComb()

        curr_comb1 =  'Servo ' +  selec_[0]  + ': '
        curr_comb2 =  'Servo ' +  selec_[1]  + ': '
        curr_comb3 =  'Servo ' +  selec_[2]  + ': '
        curr_comb4 =  'Servo ' +  selec_[3]  + ': '

        self.s1.config(text=curr_comb1)
        self.s2.config(text=curr_comb2)
        self.s3.config(text=curr_comb3)
        self.s4.config(text=curr_comb4)
        self.update_thread(ind = 75)

        return
        
    def combDown (self):
        global serv_comb
        global SelecServoVals 
        SelecServoVals = []

        comb_summ = serv_comb

        if serv_comb == 0:
            curr_text= 'Comb minima seleccionada' 
            self.status.config(text=curr_text,fg= "black")
            exit
        else:
            curr_text= 'Using Servo Combo ' + str(serv_comb)
            self.status.config(text=curr_text,fg= "black")
            comb_summ = comb_summ - 1
            exit

        serv_comb = comb_summ    
        
        selec_ = self.selecComb()

        curr_comb1 =  'Servo ' +  selec_[0]  + ': '
        curr_comb2 =  'Servo ' +  selec_[1]  + ': '
        curr_comb3 =  'Servo ' +  selec_[2]  + ': '
        curr_comb4 =  'Servo ' +  selec_[3]  + ': '

        self.s1.config(text=curr_comb1)
        self.s2.config(text=curr_comb2)
        self.s3.config(text=curr_comb3)
        self.s4.config(text=curr_comb4) 
        
        self.update_thread(ind = 75) 
        
        return
    ##TO DO: Change Max Grad
    def selecGrad(self):
        global max_grad

        try:
            max_grad = int(self.e.get())
            self.servo1.config(to=max_grad)
            self.servo2.config(to=max_grad)
            self.config(to=max_grad)
            self.config(to=max_grad)
            self.config(text='Maximum set to: '+ str(max_grad)+ ' Degrees', fg='black' )
        except:
           self.config(text='Please Enter Maximum Degree of the Servo!', fg='red')

    def selecComb(self):
    
        global selec
        selec = serv_comb
        

        if selec == 0:
            comb = ["1", "2", "3", "4"]
        elif selec == 1:
            comb = ["5", "6", "7", "8"]
        elif selec == 2:
            comb = ["9", "10", "11", "12"]
        else: 
            comb = ["13", "14", "15", "16"]

        return comb
    
    def addj(self):
        global j

        j = j + 1

        i = j - 1
        
        #print(j-1)
        return i

    def subj(self):
        global j
        j = j - 1

        #print(j-1)
        return j


    def getScale1(self,var):
        val1=self.current_val1.get()
        str_val = str(val1)
        self.master.after(12,self.update_thread(0,val1))
        self.val_s1.config(text=str_val)
        
    def getScale2(self,var):  
        val2=self.current_val2.get()
        str_val = str(val2)
        self.master.after(12,self.update_thread(1,val2))
        self.val_s2.config(text=str_val)

    def getScale3(self,var):
        val3=self.current_val3.get()
        str_val = str(val3)
        self.master.after(12,self.update_thread(2,val3))
        self.val_s3.config(text=str_val)

    def getScale4(self,var):  
        val4=self.current_val4.get()
        str_val = str(val4)
        self.master.after(12,self.update_thread(3,val4))
        self.val_s4.config(text=str_val)

    ## THREADING ##
    def process_queue(self):
        try:
            msg = self.queue.get_nowait()
            print(msg, flush= True, end="\r")
            self.status.config(text=msg, fg="green")
            if conn_bool == True:
                arduinoData.write(msg.encode())
                arduinoData.reset_output_buffer()
                arduinoData.flush()
                
        except queue.Empty:
            self.master.after(12, self.process_queue)

    def conn(self):
        global arduinoData
        global conn_bool
        if conn_bool == False:
            try:
                connectBoard()
            except:
                print("CONNECTION TO ARDUINO BOARD FAILED")
                self.status.config(text= "CONNECTION TO ARDUINO BOARD FAILED", fg="red")
            else:
                time.sleep(1)
                self.update_thread(ind = 67)
                conn_bool = True
        else:
            self.update_thread(ind = 67)
            #self.status.config(text= "Arduino already connected", fg="orange")

    def update_thread(self, ind = 0, val = 0):
        global set_ind
        global set_val
        set_ind = ind 
        set_val = val
        self.queue = queue.Queue()
        ThreadedTask(self.queue).start()
        self.master.after(12, self.process_queue) 


class ThreadedTask(threading.Thread):
    def __init__(self, queue):
        super().__init__()
        self.queue = queue
        global arduinoData
        global set_val
        self.val = set_val

    def run(self):
        comm = "!UA"
        global set_ind
        global servoVals
        global SelecServoVals
        global k 
        
        try:
            inty = int(set_val)
        except:
                self.queue.put("UNKNOWN COMMAND : " + set_val)
                
        else:
            if serv_comb == 0:
                k = 0
                SelecServoVals = servoVals[:4]
            elif serv_comb == 1:
                k = 4
                SelecServoVals = servoVals[4:8]
            elif serv_comb == 2:
                k = 8
                SelecServoVals = servoVals[8:12]
            else:
                k = 12
                SelecServoVals = servoVals[12:16] 

            if set_ind == 67:
                for x in range(150):
                    arduinoData.write('!connect\r'.encode())
                    in_line = arduinoData.readline().decode()
                    set_ind = 0
                    if in_line != "":
                        self.queue.put(in_line)
                        break
            elif set_ind == 75:
                for x in range(150):
                    in_line2 = arduinoData.readline().decode()
                    glob_servo1.config(state = "disabled")
                    glob_servo2.config(state = "disabled")
                    glob_servo3.config(state = "disabled")
                    glob_servo4.config(state = "disabled")
                    
                    try:
                        dif = int(in_line2)
                    except:
                        glob_status.config(text = "Changing Motor combination please wait. ", fg= "orange")
                        print("Changing Motor combination please wait. ",flush=True, end= "\r")
                        k_send = '!UK ' + str(k) + "\r"

                        arduinoData.write(k_send.encode())
                        set_ind = 0
                    else:
                        if dif == k:
                            glob_servo1.config(state = "normal")
                            glob_servo2.config(state = "normal")
                            glob_servo3.config(state = "normal")
                            glob_servo4.config(state = "normal")
                            self.queue.put(in_line2)
                            set_ind = 0
                            break
            else:                              
                SelecServoVals[set_ind] = set_val
                servoVals[set_ind + int(k)] = set_val  
                for x in SelecServoVals:
                    comm += " " + str(int(x))  
                comm = comm + "\r"
                self.queue.put(comm)

## GUI ##         
root = Tk()
main_ui = GUI(root)
root.mainloop()         
#ka0,9,10,90,40