#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/adc.h"
#include <ctype.h>
#include <unistd.h>
#include "pico/time.h"
#include <string.h>
#include "globalVariables.h"

#define ECHO_PIN 11
#define BARCODE_SENSOR 7
#define ARRAY_SIZE 9
#define DICTIONARY_SIZE 30
#define DEBOUNCE 20     
#define INTERVAL_US 1500000

int globalindex =0;
//bar structure to hold the voltage, time start, bar length and its color
struct bar{
    uint16_t voltage;
    absolute_time_t bar_start;
    int64_t barLength;
    int color; // black: 1 or white: 0
};

//inititalize a list of 10 bar.
struct bar barList[ARRAY_SIZE]= {0};
//key value pair for the dictionary
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

int barcode[3][9]; //1 full barcode is 3 9-bar 
char decodedBarcode[3]; //3 full barcode in maze
volatile uint64_t interrupttime = 0; 
int barcodeIndex = 0; //which bar at the current point
volatile bool timer_triggered = false;
int currentIndex = 0;

int64_t alarm_callback(alarm_id_t id, void *user_data) {
    //timer to detect if its a front wall or first bar. if pass 1.5s and no new bar, it is a wall
    printf("Timer %d fired!\n", (int) id);
    timer_triggered = true;
    // Can return a value here in us to fire in the future
    return 0;
}

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
        barList[index].voltage = result; //set voltage
        barList[index].barLength = -1; // no bar length until next bar come in, then next bar start - this bar start = length
        barList[index].color = 1;
        barList[index].bar_start = current_time; //set start time
        printf("Bar %d - Voltage: %d, Color: %s, Start: %llu, len:  %llu\n" , index, barList[index].voltage, barList[index].color == 1 ? "Black" : "White", barList[index].bar_start, barList[index].barLength); 
        return 1;
    }else{
        if(absolute_time_diff_us(barList[index-1].bar_start,current_time)>1000){
            if (barList[index-1].color == 1){
                //white
                barList[index-1].barLength = absolute_time_diff_us(barList[index-1].bar_start,current_time); // set previous bar length
                barList[index].voltage = result;
                barList[index].barLength = -1;
                barList[index].color = 0;
                barList[index].bar_start = current_time;
                printf("Bar %d - Voltage: %d, Color: %s, Start: %llu, len:  %llu\n" , index-1, barList[index-1].voltage, barList[index-1].color == 1 ? "Black" : "White", barList[index-1].bar_start, barList[index-1].barLength); 
                if(index < 8){ // for front wall detection
                    // Start the 1.5-second timer for deadend detection and reset if invalid
                    /*currentIndex = index;
                    add_alarm_in_ms(1500, alarm_callback, NULL, false);*/
                }
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
    for (int i = 0; i < 9; i++) { // sum up all thickness than divide by 9 for the average.
            total+=barList[i].barLength;
        }
    float average = total/9;   
    for (int i = 0; i < 9; i++) { //more than average is thick, less than is thin
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

void reverseString(const char *str, char *result) {
    //reverse the barcode to compare with the existing dictionary for reverse barcode functions
    int length = strlen(str);

    // Copy the original string to the result buffer
    strcpy(result, str);

    int start = 0;
    int end = length - 1;

    while (start < end) {
        char temp = result[start];
        result[start] = result[end];
        result[end] = temp;
        start++;
        end--;
    }
}

static int decodeBarCode(int barcodeIndex){
    classifyThickness(barcodeIndex);// get thickness
    char fullbarcode[10];
    int legit = 0;
    for (int i = 0; i < 9; i++) {
        char temp;
        temp = (char)(barcode[barcodeIndex][i]+48); //convert int to char,, plus 48 because of ascii stuff
        printf("%c\n", temp); 
        fullbarcode[i] = temp; 
    }
    printf("%s\n",fullbarcode);
    fullbarcode[9]='\0';
    //For Reverse Barcode
    //char reverseStrings[ARRAY_SIZE];
    //reverseString(fullbarcode, reverseStrings);
    //printf("%s\n",reverseStrings);
    //reverseStrings[9]='\0';
    for (int x = 0; x < 27; x++) {
        //printf("%s\n",dictionary[x].value);
        char *substring = malloc(ARRAY_SIZE +1 );
        strncpy(substring, dictionary[x].value, ARRAY_SIZE);        
        substring[ARRAY_SIZE] = '\0';
        // char*reverseString = malloc(ARRAY_SIZE +1);
       // char *reversedString = reverseString(fullbarcode);
        
        //printf("Substring: %s\n", substring);
        if (strcmp(substring, fullbarcode) == 0) { //compare value in dictionary to full converted barcode 
            printf("Match found for key %c\n", dictionary[x].key);
            legit = 1;
            decodedBarcode[barcodeIndex] =dictionary[x].key;
            if(decodedBarcode[barcodeIndex] != '*'){
                // Clear the message buffer
                xMessageBufferReset(xBarcodeCharHandler);
                xMessageBufferSend(xBarcodeCharHandler, (void*) &decodedBarcode[barcodeIndex], sizeof(decodeBarCode), 0);
            }
            printf("Barcode says %c",decodedBarcode[barcodeIndex]);
            printf("Barcode is valid\n");                                    
            break;
        }
        free(substring);
    }    
    if(legit==0){
        char notValid[2];
        // Copy the string "Not Valid" into the notValid array
        strcpy(notValid, "-");  
        printf("Not valid:%s\n", notValid);
        // Clear the message buffer
        xMessageBufferReset(xBarcodeCharHandler);
        xMessageBufferSend(xBarcodeCharHandler, (void*) &notValid, sizeof(notValid), 0);
        return 0;
    }else{
        return 1;
    }
}

static void readBarcode(){
    
    if(decodedBarcode[0] == '*' && decodedBarcode[2] =='*' ){ //if first and last value is * then its valid
        printf("%c %c %c",decodedBarcode[0],decodedBarcode[1],decodedBarcode[2]);
        printf("Barcode says %c",decodedBarcode[1]);        
    }else{
        printf("%c %c %c",decodedBarcode[0],decodedBarcode[1],decodedBarcode[2]);
        printf("Barcode is not valid because the first and last values are not *");              
    }
}

void interrupt_callback_barcode(uint gpio, uint32_t events) {    
    printf("enter IR interrupt \n");
        uint64_t current = to_ms_since_boot(get_absolute_time());
        if ((current - interrupttime) < DEBOUNCE)
        {
            //prevent the repeated IR interrupt
            printf("debounce\n");
            return;
        }
        interrupttime = current;
    if(gpio==BARCODE_SENSOR &&(events == GPIO_IRQ_EDGE_RISE || events == GPIO_IRQ_EDGE_FALL)){
        
        adc_select_input(0);
        uint32_t result = adc_read(); //result is a range between 0 - ~2000, above 850 is black
        globalindex = -1;
        for (int i = 0; i < ARRAY_SIZE; i++) {
            if (barList[i].voltage==0)
            {
                globalindex = i;
                break;
            }        
        }
        if(globalindex == -1 ){
            //all 9 slots are full
            if(decodeBarCode(barcodeIndex)==1){
                barcodeIndex+=1;
            }
            if(barcodeIndex == 3){
                readBarcode();
            }
            for (int i = 0; i < ARRAY_SIZE; i++) {
                barList[i].voltage = 0;
            }
        }else{
            handleBars(result,globalindex);
        }
        
    }
    // vTaskSuspendAll();    
    if (gpio == LEFT_WHEEL_ENCODER) 
    {           
        uint32_t currentTime = time_us_32();
        xMessageBufferSend(xMotorEncoderTimerHandler, (void *) &currentTime, sizeof(uint32_t), 0);
        int dataToSend = 1;
        //set the left wheel encoder to true and send it using MessageBufferHandle to main.c
        xMessageBufferSend( /* The message buffer to write to. */
                    xMotorLeftEncoderHandler,
                    /* The source of the data to send. */
                    (void *) &dataToSend,
                    /* The length of the data to send. */
                    sizeof( int ),
                    /* The block time; 0 = no block */
                    0 );
        printf("Data to send at handler %d\n", dataToSend);
    } 
    else if (gpio == RIGHT_WHEEL_ENCODER)
    {   
        uint32_t currentTime = time_us_32();
        xMessageBufferSend(xMotorEncoderTimerHandler, (void *) &currentTime, sizeof(uint32_t), 0);     
        int dataToSend = 1;
        // set the left wheel encoder to true and send it using MessageBufferHandle to main.c
        xMessageBufferSend( /* The message buffer to write to. */
                    xMotorRightEncoderHandler,
                    /* The source of the data to send. */
                    (void *) &dataToSend,
                    /* The length of the data to send. */
                    sizeof( int ),
                    /* The block time; 0 = no block */
                    0 );
        printf("Data to send at handler %d\n", dataToSend);
    }        

    if (gpio == ECHO_PIN && events == GPIO_IRQ_EDGE_RISE) {  // Rising edge
        start_time_us = to_us_since_boot(get_absolute_time());        
    } else if(gpio == ECHO_PIN && events == GPIO_IRQ_EDGE_FALL){  // Falling edge
        end_time_us = to_us_since_boot(get_absolute_time());
        echo_received = true;
    }
}

void barcodeTask(){
    init_barcode_pins();
    adc_init();
    gpio_set_irq_enabled_with_callback(BARCODE_SENSOR, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, &interrupt_callback_barcode);
    while(1){
        if (timer_triggered) {
            /*if (barList[2].voltage == 0) {
                // wall handler. if bar 2 is still 0 = wall, so reset all bars
                barList[0].voltage == 0;
                barList[0].barLength == 0;
                barList[1].voltage == 0;
                //turn
                printf("deadend!\n");
            }*/
            // Reset the flag
            timer_triggered = false;
        }
        tight_loop_contents();
        vTaskDelay(103);
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
