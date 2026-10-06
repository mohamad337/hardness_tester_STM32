- refrence voltage is 2.5V
- 16 chanel of ADC for Thermocouple (pc0-pc5,pa0-pa7,pf4,pb0-pb2,pb12-pb15) (0-2.5v means 0-30000um)
- dc motor driver (L298 ,IN1=PH6,IN2=PH7,Enable=PH8)
- deep meter on ADC1 input 19 in 16bit resulotion (0-3v means 0-30000um)
- power meter on ADC1 input 16 (input power voltage 0-3v means 0-30v)
- force mesurment from HX711 on SPI2 
- flash memory is W25Q256FVEI on SPI4
for measure h deep in brinell hardness test:
1- when PD11 change to 1 (start messurment) steper motor move up until zero (PD11 == 1 or PD12 ==1)  
2- in this point force muste be equal to 0 else if force<5 get frce=0 else  returne error
3- get PA5 FOR h-deep meter then move down (in speed fast) and counte steper motor pulse untile near of block (20000 pulse)
4- then steper motor move down in low speed untile force mesurment from HX711 a few changed.
5- then stepper motor stop and direction change and motor up untile force mesurment = 0.
6- this point is statr position and for h-deep meter is zero position
7- then direction change and motor down untile force mesurment from HX711 equalt to set point saved on flash in adress 1
8- then waite in secound ( saved on flash in address 4).
9- then calcluat (h-deep now - h-deep zero) and (now stepper counte) and steper count in start poin and send on the RS232 port (UART4) and Modbus RTU (USART1)
10-run on right dc motor in 10 secount and take a picture (PD2=1 in one secund and then 0)
11-run on left dc motor in 10 secund 
12- move up stepper motor untile zero point