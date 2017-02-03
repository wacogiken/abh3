# abh3
ROS Package for [ABH3](https://www.wacogiken.co.jp/agv/abh3.html) Driver


![](img/img_abh3_01.jpg)

**_Version History_**
* __v1.0__(2016/10/28)
 * First release
* __v1.1__(2017/02/02)
 * abh3_com.py
   * Append DOUT / DIN / ID command.
   * Measures to prevent communication errors at power off.
 * abh3_joy.py
   * Output speed zero when command button is off.
