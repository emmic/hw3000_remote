void HAL_Delay_nMS(unsigned int time_delay)
{
 unsigned int i,j;
 for(i=time_delay;i>0;i--)
 {
   for(j=400;j>0;j--);
 }
}

void HAL_Delay_nNS(unsigned int time_delay)
{
 unsigned int i,j;
 for(i=time_delay;i>0;i--)
 {
   for(j=3;j>0;j--);
 }
}