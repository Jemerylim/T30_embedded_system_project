#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/adc.h"
#include <ctype.h>
#include <unistd.h>
#include "pico/time.h"
#include <string.h>

#define DIGITAL_PIN 22
#define ARRAY_SIZE 10
#define DICTIONARY_SIZE 30

struct bar{
    uint16_t voltage;
    absolute_time_t bar_start;
    int64_t barLength;
    int color; // black: 1 or white: 0
};
struct bar barList[ARRAY_SIZE];

struct conversion {
    char key;
    char value[ARRAY_SIZE];
};
struct conversion dictionary[DICTIONARY_SIZE] = {
    {'A',"031312130"},
    {'B',"130312130"},
    {'C',"030312131"},
    {'D',"131302130"},
    {'E',"031302131"},
    {'F',"130302131"},
    {'G',"131312030"},
    {'H',"031312031"},
    {'I',"130312031"},
    {'J',"131302031"},
    {'K',"031313120"},
    {'L',"130313120"},
    {'M',"030313121"},
    {'N',"131303120"},
    {'O',"031303121"},
    {'P',"130303121"},
    {'Q',"131313020"},
    {'R',"031313021"},
    {'S',"130313021"},
    {'T',"131303021"},
    {'U',"021313130"},
    {'V',"120313130"},
    {'W',"020313131"},
    {'X',"121303130"},
    {'Y',"021303131"},
    {'Z',"120303131"},
    {'*',"121303031"}
};

int barcode[3][9];
char decodedBarcode[3];

/*
thickness classification
thick black 0
thin black 1
thick white 2
thin white 3
*/


static int isBlackOrWhite(uint32_t result){
    int threshold = 800; //more than 800 is black
    if (result >= threshold || gpio_get(DIGITAL_PIN) == 1) { //black
        return 1;
    }else{
        return 0;
    }
}

static int handleBars(uint32_t result, int index){
    int blackOrWhite = isBlackOrWhite(result); //get color
    absolute_time_t current_time = get_absolute_time(); //get start time
    if(index == 0){
        if(blackOrWhite == 1){ //only add if its black
            barList[index].voltage = result;
            barList[index].barLength = -1;
            barList[index].color = blackOrWhite;
            barList[index].bar_start = current_time; //set start time
            return 1;
        }else{// barcode has not start
            return 0;
        }
    }else{
        if(barList[index-1].color != blackOrWhite){// if bar is not the same color as the last one saved
            if(index == 1 || index == 9){
                int64_t length =  absolute_time_diff_us(barList[index-1].bar_start,current_time);
                barList[index-1].barLength = length - 20000; //adjusted for some anomaly values
            }else{
                barList[index-1].barLength = absolute_time_diff_us(barList[index-1].bar_start,current_time); //minus current time with last bar start time to get length
            }
            barList[index].voltage = result;
            barList[index].barLength = -1;
            barList[index].color = blackOrWhite;
            barList[index].bar_start = current_time;
            return index + 1;
        }else{
            return index;
        }
    }
}

static void classifyThickness(int barcodeIndex){
    int total = 0;
    for (int i = 0; i < 10; i++) { // 2 black one white
            total+=barList[i].barLength;
        }
    float average = total/9;   
    for (int i = 0; i < 10; i++) {
        if (barList[i].barLength >= average)//thick
        {
            if(barList[i].color == 1){//black
                barcode[barcodeIndex][i] = 0;
            }else{//white
                barcode[barcodeIndex][i] = 2;
            }
        }else{//thin
            if(barList[i].color == 1){//black 
                barcode[barcodeIndex][i] = 1;
            }else{//white
                barcode[barcodeIndex][i] = 3;
            }
        }
        //printf("%d",barcode[barcodeIndex][i]);    
    }
}
static int decodeBarCode(int barcodeIndex){
    classifyThickness(barcodeIndex);// get thickness
    char fullbarcode[10];
    int legit = 0;
    for (int i = 0; i < 9; i++) {
        printf("%d",barcode[barcodeIndex][i]);//print full list of bar thickness
    }
    for (int i = 0; i < 9; i++) {
        char temp;
        temp = (char)(barcode[barcodeIndex][i]+48); //convert int to char
        printf("%c\n", temp); 
        fullbarcode[i] = temp; 
    }
    fullbarcode[9]='\0';
    printf("%s\n",fullbarcode);
    for (int i = 0; i < 28; i++) {
        if (strcmp(dictionary[i].value, fullbarcode) == 0) { //compare value in dictionary to full converted barcode 
            printf("Match found for key %c\n", dictionary[i].key);
            legit = 1;
            decodedBarcode[barcodeIndex] =dictionary[i].key;
            break;
        }
    }
    if(legit==0){
        printf("Not valid\n"); 
        return 0;
    }else{
        return 1;
    }
}

static void readBarcode(){
    if(decodedBarcode[0] == '*' && decodedBarcode[2] =='*' ){
        printf("%c %c %c",decodedBarcode[0],decodedBarcode[1]decodedBarcode[2]);
        printf("Barcode says %c",decodedBarcode[1]);
    }else{
        printf("%c %c %c",decodedBarcode[0],decodedBarcode[1]decodedBarcode[2]);
        printf("Barcode is not valid because the first and last values are not *");
    }
}

int main() {
    stdio_usb_init();
    adc_init();
    adc_select_input(0);
    int barcodeIndex =0;
    int index = 0;
    while (1) {
        uint32_t result = adc_read(); //result is a range between 0 - ~2000, above 850 is black
        // adc_select_input(1); // ADC1 GP27
        // uint32_t result_left = adc_read(); // voltage above 1 should be black
        // const float conversion_factor = 3.3f / (1 << 12);
        // printf("\n0x%03x -> %f V\n", result_left, result_left * conversion_factor);
        // adc_select_input(2); // ADC2 GP28
        // uint32_t result_right = adc_read();
        // printf("\n0x%03x -> %f V\n", result_right, result_right * conversion_factor);
        index = handleBars(result,index);
        for (int i = 0; i < 10; i++) {
            printf("Bar %d - Voltage: %d, Color: %s, Start: %llu, len:  %llu\n" , i + 1, barList[i].voltage, barList[i].color == 1 ? "Black" : "White", barList[i].bar_start, barList[i].barLength);
        }
       if(index>9){
            if(decodeBarCode(barcodeIndex)==0){
                index =0;
                //sleep_ms(3000);
            }else{
                if(barcodeIndex < 3){
                    index =0;
                    printf("%c",decodedBarcode[barcodeIndex]);
                    barcodeIndex+=1;
                    sleep_ms(3000);
                }else{
                    printf("done");
                    readBarcode();
                    index =0;
                    barcodeIndex=0;
                    sleep_ms(3000);
                }
            }
        }
    }
}
