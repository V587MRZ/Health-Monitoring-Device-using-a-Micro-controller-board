# Health-Monitoring-Device-using-a-Micro-controller-board
Health Monitoring Device to detect heart rate, IBI and BPM  
Onboard Signal Processing / Signal Analysis:  

The final product should have the following functions:  

(a) The signal from the pulse sensor is displayed continuously on your computer screen
or an alternate display of your choice. 

(b) The average heart rate (i.e., the number of heart beats per minute) is continuously
calculated on the microcontroller and shown on the display.  

(c) The inter-beat interval (i.e., the time between two beats) is continuously calculated on
the microcontroller and displayed.  

(d) The total number of heart beats is counted and displayed.  

(e) If the user presses the blue push button, the calculations in part (b) and (d) are reset.  

Output / Display:
Once again, you may display the processed data however you like (see the audio display
section above). We recommend using UART to transmit the processed data to your computer
to be displayed. The display must clearly show the results from (a), (b), (c) and (d). Note that
the outputs of (a), (c) and (d) are updated continuously while (b) is updated when appropriate.  
Advanced Activities  


(i) In addition to computer screen, you could use the LCD display (from the part list)
with I2C functionality to display the total number of heart beats (d).  

(ii) Display a warning if no heart beats have been detected for some time. This could
simply be a LED connected to the microcontroller.
