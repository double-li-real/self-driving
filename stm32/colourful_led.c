无人车前后跑马灯代码：修改colour_led.c中L_runningled部分
void L_runingled(void)    
{
    u8 i;
    u8 j;
    while(1)
    {
        for(i=1; i<=led_num; i++)
        {
            for(j=1; j<=led_num; j++)
            {
							 if(i==j)
                 L_ws2812_rgb(j, WS_WHITE );
							 else
								 L_ws2812_rgb(j, WS_DARK );
               R_ws2812_rgb(j, WS_DARK);
            }

            L_ws2812_refresh(led_num);
            R_ws2812_refresh(led_num);
            delay_ms(400);
        }

        for(i=led_num; i>0; i--)
        {
            for(j=1; j<=led_num; j++)
            {
               if(i==j)
                R_ws2812_rgb(j, WS_WHITE );
							 else
								 R_ws2812_rgb(j, WS_DARK );
               L_ws2812_rgb(j, WS_DARK);
            }
						 R_ws2812_refresh(led_num);
						L_ws2812_refresh(led_num);
            delay_ms(400);
        }
    }
}
