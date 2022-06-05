/*
 * Weighing_scale1.c
 *
 *  Created on: 04-Jun-2022
 *      Author: Jiyansh
 */

#include "Weighing_scale.h"
#include "main.h"

void calibration(int channel)
{
	int i;
	unsigned long int val=0;
	//counts without any load
	for(i=0;i<20;i++)
		BaseCount.value=avg(channel);
	//display "LOAD" on screen
	//Display("LOAD");
	HAL_Delay(500);
	//while(HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_14)!=0);
	//weight in grams for calibration read from display(user)
	//loadd=Read_display();//edit(000000);
	//counts after keeping load
	for(i=0;i<20;i++)
		CurrentCount=avg(channel);
	//calibration factor(counts per gram)
	val=CurrentCount-BaseCount.value;
	calibration_facotr.cf1=(float)val/(float)loadd;
	//save calibration factor in memory for weight calculation
	//save();
}

long int avg(int channel)
{
	int i;
	unsigned long int ADC_Counts,Avg_ADC_Counts=0;
	for(i=0;i<30;i++)
	{
		ADC_Counts=read7802(channel);
		Avg_ADC_Counts=Avg_ADC_Counts+ADC_Counts;
	}
	return(Avg_ADC_Counts/300);
}

void calculation(int channel)
{
	CurrentCount=avg(channel);
	if(CurrentCount>BaseCount.value)
	{
		diff=CurrentCount-BaseCount.value;
		neg_f=0;
	}
	else
	{
		diff=BaseCount.value-CurrentCount;
		neg_f=1;
	}
	weight=diff/calibration_facotr.cf1;
	wt_f=(float)(diff)/(calibration_facotr.cf1);
	wt_f=wt_f-weight;
	weight=weight/acc;
	weight=weight*acc;
}
void overload(void)
{
	if(weight>cap.value)
	{
		//overload msg on Display
		//Display("Overload");
		dp_flag=0;
	}
	else
	{
		dp_flag=1;
		//display weight on display
		//display(weight);
	}
}

void accuracy(void)
{
	if(wt_f>(0.5*acc))
		weight=weight+acc;
	else
		weight=weight;
}
