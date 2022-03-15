<h1>Servo Motor Driver</h1>
<p>STM32 HAL & LL library for driving RC Servo motor.</p>
<p>Include standard and high resolution timer control method.</p>
<p>This example is using uart to control the rotatation of the servo.</p>

## <ins>Concept</ins>
<p>Standard way to driver servo is to configure pwm signal as below:</p>
<img src="Images/pulse.png" width="40%" height="40%">
<p>but some servo do had different requirement.</p>
<p>Here i tested MG90s Micro servo where i got it from taobao.</p>
<img src="Images/MG90S.jpg" width="40%" height="40%">
<img src="Images/MG90S_180_datasheet.jpg" width="60%" height="60%">
<p>Base on the data sheet, it need pulse of 500-2500us to rotate 0-180 deg.</p>

## <ins>Calculation</ins>
<p>First we using normal single timer to generate PWM pulse as per requirement.</p>

<img src="Images/Timer16.jpg" width="40%" height="40%">

<p>In order to get 50Hz pwm frequency, here is the calculation:</p>

<!-- $$
Fpwm = \frac{Ftim}{(PSC * ARR)}
$$ --> 
<div align="left"><img style="background: white;" src="Images\ALb5F9z7ms.png"></div>

<!-- $Fpwm = \frac{64MHz}{((19+1) * (63999+1))} = 50Hz$ -->
<div align="left"><img style="background: white;" src="Images\pvijEdmHRB.png"></div>

<p>The datasheet show the servo turn to 0deg @ 0.5ms and 180deg @ 2.5ms,</p>
<p>From here we calculate 90deg should be around 4800</p>

<!-- $CCR = ARR * 7.5\%$ --> 
<div align="left"><img style="background: white;" src="Images\1AORoKJFL6.png"></div>  

<!-- $CCR = 64000 * 0.075 = 4800$ --> <img style="transform: translateY(0.1em); background: white;" src="Images\JdKC3u8xTf.png">

(10% = 2ms, 7.5% = 1.5ms, 5% = 1ms)

## <ins>Usage</ins>
<p>Com Port baudrate: 115200</p>
<p>To find max and min S1 servo pulse:</p>

``CAL S1 1850``

<p>To set S1 rotatation angle 45deg</p>

``ROT S1 45``