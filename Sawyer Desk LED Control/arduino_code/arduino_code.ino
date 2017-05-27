
#include <elapsedMillis.h>
#include <led_demo/led.h>
/* 
 * rosserial Subscriber Example
 * Blinks an LED on callback
 */

#include <ros.h>

class zone
{
 public:
   zone(int pin_R, int pin_G, int pin_B);
   void set_rgb(int r, int g, int b, int rnd);
   void set_blink(boolean yes, int rate_ms);
   void set_self_control(boolean yes);
   void set_on(boolean yes);
   
   void perform_task();
 
 private://members
   int _pin_R;
   int _pin_G;
   int _pin_B;
   
   boolean first;
   
   elapsedMillis time;
   
   boolean _ON;
   boolean _blink;
   boolean _self_control;
   int _rate;
   boolean _sub_off;
   boolean flag;
   
   boolean _rand_colour;
   boolean _switch;
   
   byte _r;
   byte _g;
   byte _b;
   
   float _h;
   float _s;
   float _v;
   
   float brightness;
   
 private://methods
   void rgb2hsv();
   void hsv2rgb();
   void blink_task();
   void breath_task();
   void off();
   void set_colour();
   
 };

zone::zone(int pin_R, int pin_G, int pin_B)
{
   _pin_R = pin_R; 
   _pin_G = pin_G;
   _pin_B = pin_B;
   
   pinMode(_pin_R, OUTPUT);
   pinMode(_pin_G, OUTPUT);
   pinMode(_pin_B, OUTPUT);
   
   time = 0;
   
   _sub_off = false;
   
   brightness = (float)0.0;
   
   flag = false;
   
   _ON = false;
   _self_control = false;
   
   first = true;
   _rand_colour = false;
   
}

void zone::set_on(boolean yes)
{
  _ON = yes;
}  

void zone::set_rgb(int r, int g, int b, int rnd)
{
   _r = r;
   _g = g;
   _b = b; 
   
   if(rnd == 1)
   {
     _rand_colour = true;
     _switch = false;
   }
   else
   {
     _rand_colour = false;
   }
   
   rgb2hsv();
}

void zone::set_blink(boolean yes, int rate_ms)
{
   _blink = yes;
   _rate = rate_ms;
   
   if (!_blink)
   {
     first = true;
   }
 
}

void zone::set_self_control(boolean yes)
{
   _self_control = yes;
}

void zone::perform_task()
{
   if(_ON == false)
   {
      off();
      return;
   }
   else
   {
     if(_self_control)
     {
         set_colour();
         return;
     }
     else 
     {
         if(_blink)
         {
           
           if (time > _rate && _sub_off)
           {
             set_colour();
             time = 0;
             _sub_off = false;
           }
           else if(time > _rate && !_sub_off)
           {
             off();
             time = 0;
             _sub_off = true;
           }
         }
         else
         {
           if (time > (_rate/40) && !flag)
           {
             if(_rand_colour && _switch)
             {
               set_rgb(random(255), random(255), random(255), 1);
               _switch = false;
             }
             if(first)
             {
                _v = 0.0;
                first = false;
             } 
             
             _v = (float)(_v + (float)(1.0/20.0));
             
             if (_v >= (float)1)
             {
               flag = true;
             }
               
             hsv2rgb();
             set_colour();
             time = 0;
           } 
           else if (time > (_rate/40) && flag)
           {
             _v = (float)(_v - (float)(1.0/20.0));                            
             
             if (_v <= (float)0)
             {
               flag = false;
               _switch = true;
             }
             
             hsv2rgb();
             set_colour();
             time = 0;
           }
         }
     } 
   }
}



void zone::set_colour()
{ 
  analogWrite(_pin_R, _r);
  analogWrite(_pin_G, _g);
  analogWrite(_pin_B, _b);
}

void zone::off()
{
  analogWrite(_pin_R, 0);
  analogWrite(_pin_G, 0);
  analogWrite(_pin_B, 0);
}

void zone::hsv2rgb()
{
  double r, g, b;

  int i = int(_h * 6);
  double f = _h * 6 - i;
  double p = _v * (1 - _s);
  double q = _v * (1 - f * _s);
  double t = _v * (1 - (1 - f) * _s);
  
  switch(i % 6){
      case 0: r = _v, g = t, b = p; break;
      case 1: r = q, g = _v, b = p; break;
      case 2: r = p, g = _v, b = t; break;
      case 3: r = p, g = q, b = _v; break;
      case 4: r = t, g = p, b = _v; break;
      case 5: r = _v, g = p, b = q; break;
  }
  
  _r = r * 255;
  _g = g * 255;
  _b = b * 255;
}

void zone::rgb2hsv()
{
    double rd = (double) _r/255;
    double gd = (double) _g/255;
    double bd = (double) _b/255;
    double max_val = max(max(rd, gd), bd), min_val = min(min(rd, gd), bd);
    double h, s, v = max_val;
    
    double d = max_val - min_val;
    s = max_val == 0 ? 0 : d / max_val;
    
    if (max_val == min_val) { 
        h = 0; // achromatic
    } else {
        if (max_val == rd) {
            h = (gd - bd) / d + (gd < bd ? 6 : 0);
        } else if (max_val == gd) {
            h = (bd - rd) / d + 2;
        } else if (max_val == bd) {
            h = (rd - gd) / d + 4;
        }
        h /= 6;
    }
    
    _h = h;
    _s = s;
    _v = v;
}

ros::NodeHandle  nh;

zone *zones;

void messageCb( const led_demo::led& toggle_msg){
  
  for(int i = 0; i < toggle_msg.zone_length; i++)
  {
    
    if(toggle_msg.zone[i] > 1)
    {
      continue;
    }
    
    if(toggle_msg.rgb_length != 4)
    {
      break;
    }
    zones[toggle_msg.zone[i] - 1].set_rgb(toggle_msg.rgb[0], toggle_msg.rgb[1], toggle_msg.rgb[2], toggle_msg.rgb[3]);
    zones[toggle_msg.zone[i] - 1].set_blink(toggle_msg.blink, toggle_msg.rate);
    zones[toggle_msg.zone[i] - 1].set_self_control(toggle_msg.self_control); 
    zones[toggle_msg.zone[i] - 1].set_on(toggle_msg.ON);
  }
}

ros::Subscriber<led_demo::led> sub("/sawyer/base/1", &messageCb );

void setup()
{ 
  pinMode(13, OUTPUT);
  
  nh.getHardware()->setBaud(115200);
  nh.initNode();
  nh.subscribe(sub);
  
  randomSeed(analogRead(0));
  
}

void loop()
{  
  zone z1(3,5,6);
  
  zone temp[] = {z1};
  
  zones = temp;
  while(1)
  {
    nh.spinOnce();
    
    zones[0].perform_task();
  }
}

