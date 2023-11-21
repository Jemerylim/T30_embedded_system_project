#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/adc.h"
#include <ctype.h>
#include <unistd.h>
#include "pico/time.h"
#include <string.h>

#define BARCODE_SENSOR 7
#define ARRAY_SIZE 10
#define DICTIONARY_SIZE 30
#define DEBOUNCE 100

struct bar{
    uint16_t voltage;
    absolute_time_t bar_start;
    int64_t barLength;
    int color; // black: 1 or white: 0
};

struct bar barList[ARRAY_SIZE]= {0};

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
volatile uint64_t interrupttime = 0;
int barcodeIndex = 0;


/*
thickness classification
thick black 0
thin black 1
thick white 2
thin white 3
*/

void init_barcode_pins() {
    gpio_init(BARCODE_SENSOR);
    gpio_set_dir(BARCODE_SENSOR, GPIO_IN);
}

static int handleBars(uint32_t result, int index){
    absolute_time_t current_time = get_absolute_time(); //get start time
    if(index == 0){
        //only add if its black
        barList[index].voltage = result;
        barList[index].barLength = -1;
        barList[index].color = 1;
        barList[index].bar_start = current_time; //set start time
        printf("Bar %d - Voltage: %d, Color: %s, Start: %llu, len:  %llu\n" , index, barList[index].voltage, barList[index].color == 1 ? "Black" : "White", barList[index].bar_start, barList[index].barLength); 
        return 1;
    }else{
        if(absolute_time_diff_us(barList[index-1].bar_start,current_time)>1000){
            if (barList[index-1].color == 1){
                //white
                barList[index-1].barLength = absolute_time_diff_us(barList[index-1].bar_start,current_time);
                barList[index].voltage = result;
                barList[index].barLength = -1;
                barList[index].color = 0;
                barList[index].bar_start = current_time;
                printf("Bar %d - Voltage: %d, Color: %s, Start: %llu, len:  %llu\n" , index-1, barList[index-1].voltage, barList[index-1].color == 1 ? "Black" : "White", barList[index-1].bar_start, barList[index-1].barLength); 
                return 0;
            }else{
                //black
                barList[index-1].barLength = absolute_time_diff_us(barList[index-1].bar_start,current_time);
                barList[index].voltage = result;
                barList[index].barLength = -1;
                barList[index].color = 1;
                barList[index].bar_start = current_time;
                printf("Bar %d - Voltage: %d, Color: %s, Start: %llu, len:  %llu\n" , index-1, barList[index-1].voltage, barList[index-1].color == 1 ? "Black" : "White", barList[index-1].bar_start, barList[index-1].barLength); 
                return 1;
            }
        }else{
            return 0;
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
            printf("DecodedBarcode: %c\n", decodedBarcode[barcodeIndex]);
            xMessageBufferSend(xBarcodeCharHandler, (void*) &decodedBarcode[barcodeIndex], sizeof(decodeBarCode), 0);
            break;
        }
    }
    if(legit==0){
        char notValid[11];
        // Copy the string "Not Valid" into the notValid array
        strcpy(notValid, "Not Valid");  
        printf("Not valid:%s\n", notValid);
        xMessageBufferSend(xBarcodeCharHandler, (void*) &notValid, sizeof(notValid), 0);
        return 0;
    }else{
        return 1;
    }
}

static void readBarcode(){
    
    if(decodedBarcode[0] == '*' && decodedBarcode[2] =='*' ){
        printf("%c %c %c",decodedBarcode[0],decodedBarcode[1],decodedBarcode[2]);
        printf("Barcode says %c",decodedBarcode[1]);        
    }else{
        printf("%c %c %c",decodedBarcode[0],decodedBarcode[1],decodedBarcode[2]);
        printf("Barcode is not valid because the first and last values are not *");              
    }
}

void interrupt_callback_barcode(uint gpio, uint32_t events) {
    uint64_t current = to_ms_since_boot(get_absolute_time());
    if ((current - interrupttime) < DEBOUNCE)
    {
        return;
    }
    interrupttime = current;
    if(gpio==BARCODE_SENSOR){
        adc_select_input(0);
        uint32_t result = adc_read(); //result is a range between 0 - ~2000, above 850 is black
        int index = -1;
        for (int i = 0; i < ARRAY_SIZE; i++) {
            if (barList[i].voltage==0)
            {
                index = i;
                break;
            }        
        }
        if(index == -1 ){
            //all 9 slots are full
            if(decodeBarCode(barcodeIndex)==1){
                barcodeIndex+=1;
            }
            for (int i = 0; i < ARRAY_SIZE; i++) {
                barList[i].voltage = 0;
            }
        }else{
            handleBars(result,index);
        }
        
    }

}

// int main() {
//     stdio_usb_init();
//     init_pins();
//     adc_init();
//     gpio_set_irq_enabled_with_callback(BARCODE_SENSOR, GPIO_IRQ_EDGE_RISE, true, &interrupt_callback);
//     while (true) {
//             tight_loop_contents();
//         }
// }
