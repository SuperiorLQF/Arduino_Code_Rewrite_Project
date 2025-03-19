# Arduino_Code_Rewrite_Project
main branch is for conditioning task
lever branch is for lever task

!!!在动代码前想清楚，这么写是有原因的，否则可能会重蹈一遍之前踩过的坑。    
举个栗子：    
1.将conditioning的condition task函数写成一个函数，想要保证每个loop 1ms，这是不现实的，因为在delay到1000us的中间，会被中断打断（timer1,2,3），这样delay就超过1000us（1ms）了。     
2.修改代码时请遵照现在的`pin_information`和`pin_driver`方案，因为pin_driver会检测数值有没有发生变化决定是否驱动，否则可能导致一些多驱动和重复驱动问题，比如关于laser和elec，在pin_driver里面检测从off->on会开启定时器，否则就会重复开启定时器导致定时器一直被复位。         
3.对于laser和elec，采用的方案是enable信号包络的方案。即给一个enable信号，来表示它的使能时段，然后再使能时段里调用timer自动驱动laser和elec。这样的好处是足够统一，让pin_driver可以对应不同类型的pin，提高代码的抽象程度。     

