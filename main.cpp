#include "mbed.h"
#include "LCD_DISCO_F429ZI.h"
#include "math.h"

// --- Register Addresses and Configuration Values ---
#define CTRL_REG1 0x20               // Control register 1 address
#define CTRL_REG1_CONFIG 0b01'10'1'1'1'1  // Configuration: ODR=200Hz, Enable X/Y/Z axes, power on
#define CTRL_REG4 0x23               // Control register 4 address
#define CTRL_REG4_CONFIG 0b0'0'01'0'00'0  // Configuration: High-resolution, 500dps sensitivity
#define CTRL_REG3 0x22                  // Address of Control Register 3
#define CTRL_REG3_CONFIG 0b0'0'0'0'1'000 // Enable data-ready interrupt


#define SPI_FLAG 1              // SPI communication completion flag
#define BUTTON_FLAG 2       // Event flag for user button to record passkey
#define TICKER_FLAG 3
#define DATA_READY_FLAG 4
#define DATA_BUFFER_SIZE 200
#define FILTER_COEFFICIENT 0.1f

#define OUT_X_L 0x28

#define DPS_PER_DIGIT (17.5f/1000.0f)

uint16_t passkey_x[DATA_BUFFER_SIZE], passkey_y[DATA_BUFFER_SIZE], passkey_z[DATA_BUFFER_SIZE], 
            attempt_x[DATA_BUFFER_SIZE], attempt_y[DATA_BUFFER_SIZE], attempt_z[DATA_BUFFER_SIZE];

float passkey_dps_x[DATA_BUFFER_SIZE], passkey_dps_y[DATA_BUFFER_SIZE], passkey_dps_z[DATA_BUFFER_SIZE], 
        attempt_dps_x[DATA_BUFFER_SIZE], attempt_dps_y[DATA_BUFFER_SIZE], attempt_dps_z[DATA_BUFFER_SIZE];

float dtw_matrix[DATA_BUFFER_SIZE][DATA_BUFFER_SIZE];
float dtw_threshold_x, dtw_threshold_y, dtw_threshold_z;

typedef enum{
    IDLE_STATE,
    RECORDKEY_STATE,
    ENTERKEY_STATE,
    PROCESS_STATE
} state;

EventFlags flags;       // EventFlags object to synchronize asynchronous SPI transfers

InterruptIn button(BUTTON1);
DigitalOut led_red(LED2);
DigitalOut led_green(LED1);

Ticker sampler_ticker;

LCD_DISCO_F429ZI lcd;

InterruptIn int2(PA_2, PullDown);   // Initialize INT2 pin with pull-down resistor


void button_cb(){
    flags.set(BUTTON_FLAG);
}
void spi_cb(int event) {
    flags.set(SPI_FLAG);
}
void ticker_cb(){
    flags.set(TICKER_FLAG);
}
void data_cb()
{
    flags.set(DATA_READY_FLAG);
}

float raw_to_angVel(uint16_t raw_val){
    if((raw_val & 0x8000) == 0x8000){
        return ((float)((raw_val ^ 0xFFFF) + 1)) * DPS_PER_DIGIT * (-1);
    } else{
        return ((float)raw_val) * DPS_PER_DIGIT;
    }
}

void msg_frontend(string *lcd_buff, int buff_len){
    uint16_t midX = lcd.GetXSize() / 2;
    uint16_t midY = lcd.GetYSize() / 2;
    int startX = 10;
    for(int i = 0; i<buff_len; i++){
        for(int j = 0; j<(int)(lcd_buff[i].length()); j++){
            lcd.DisplayChar(startX+j*10, midY-(70-i*20), lcd_buff[i][j]);
        }
    }
}

void unlock_frontend(){
    lcd.Clear(LCD_COLOR_BLACK);

    // Draw the face (yellow circle)
    lcd.SetTextColor(LCD_COLOR_YELLOW);
    lcd.FillCircle(120, 120, 80);

    // Draw the eyes (black circles)
    lcd.SetTextColor(LCD_COLOR_BLACK);
    lcd.FillCircle(90, 90, 10);  // Left eye
    lcd.FillCircle(150, 90, 10); // Right eye

    // Draw the smile (red arc)
    lcd.SetTextColor(LCD_COLOR_RED);
    Point smile[] = {{80, 140}, {120, 160}, {160, 140}};
    lcd.FillPolygon(smile, 3);
}

float min_num(float num1, float num2, float num3){
    float min = num1;
    if (num2 < min) {
        min = num2;
    }
    if (num3 < min) {
        min = num3;
    }
    return min;
}

void DTW(float (*dtw_matrix)[DATA_BUFFER_SIZE], float *passkey_dps, float *attempt_dps){
    for(int i = 0; i < DATA_BUFFER_SIZE; i++){
        for(int j = 0; j < DATA_BUFFER_SIZE;j++){
            if((i==0) & (j==0)){
                dtw_matrix[i][j] = fabsf(passkey_dps[i] - attempt_dps[j]);
            }else if(i==0){
                dtw_matrix[i][j] = fabsf(passkey_dps[i] - attempt_dps[j]) + dtw_matrix[i][j-1];
            }else if(j==0){
                dtw_matrix[i][j] = fabsf(passkey_dps[i] - attempt_dps[j]) + dtw_matrix[i-1][j];
            }else{
                dtw_matrix[i][j] = fabsf(passkey_dps[i] - attempt_dps[j]) + min_num(dtw_matrix[i-1][j-1], dtw_matrix[i-1][j], dtw_matrix[i][j-1]);
            }
        }
    }
}
float back_track(float (*dtw_matrix)[DATA_BUFFER_SIZE]){
    int row_min = DATA_BUFFER_SIZE-1;
    int col_min = DATA_BUFFER_SIZE-1;
    int temp_row = 0, temp_col = 0;
    int count = 0;
    float cavg = 0;
    while(1){
        cavg += dtw_matrix[row_min][col_min];
        float back_min = dtw_matrix[row_min][col_min-1];
        temp_row = row_min;
        temp_col = col_min-1;
        if(dtw_matrix[row_min-1][col_min-1] < back_min){
            temp_row = row_min-1;
            temp_col = col_min-1;
            back_min = dtw_matrix[temp_row][temp_col];
        }
        if(dtw_matrix[row_min-1][col_min] < back_min){
            temp_row = row_min-1;
            temp_col = col_min;
            back_min = dtw_matrix[temp_row][temp_col];
        }
        row_min = temp_row;
        col_min = temp_col;
        count++;
        if((row_min == 0) | (col_min == 0)){
            cavg += dtw_matrix[row_min][col_min];
            count ++;
            break;
        }
    }
    return (cavg/count);
}
    
int main() {

    uint8_t write_buf[32], read_buf[32];
    int index = 0;

    SPI spi(PF_9, PF_8, PF_7, PC_1, use_gpio_ssel);
    string idle_buff[] = {"Press Button", "To Record Gesture :)"};
    int idle_buff_len = sizeof(idle_buff)/sizeof(idle_buff[0]);
    string record_buff[] = {"Record", "The Pass Key!!!"};
    int rec_buff_len = sizeof(record_buff)/sizeof(record_buff[0]);
    string record_buff_2[] = {"Pass Key Recorded...", "Press Button To Insert", "Gesture"};
    int rec_buff_len_2 = sizeof(record_buff_2)/sizeof(record_buff_2[0]);
    string enter_buff[] = {"Enter", "The Gesture Key!"};
    int ent_buff_len = sizeof(enter_buff)/sizeof(enter_buff[0]);
    string enter_buff_2[] = {"Attempt Registered", "Press Button To Unlock"};
    int ent_buff_len_2 = sizeof(enter_buff_2)/sizeof(enter_buff_2[0]);

    spi.format(8, 3);
    spi.frequency(1'000'000);

    write_buf[0] = CTRL_REG1;
    write_buf[1] = CTRL_REG1_CONFIG;
    spi.transfer(write_buf, 2, read_buf, 2, spi_cb);  // Initiate SPI transfer
    flags.wait_all(SPI_FLAG);  // Wait until the transfer completes

    write_buf[0] = CTRL_REG4;
    write_buf[1] = CTRL_REG4_CONFIG;
    spi.transfer(write_buf, 2, read_buf, 2, spi_cb);  // Initiate SPI transfer
    flags.wait_all(SPI_FLAG);  // Wait until the transfer completes
    
    state CurrentState = IDLE_STATE;

    while (1) {
        switch(CurrentState){
            case IDLE_STATE:
                lcd.Clear(0xFFFFFFFF);
                lcd.SetBackColor(LCD_COLOR_CYAN);
                led_red = 0;
                led_green = 0;
                msg_frontend(idle_buff, idle_buff_len);
                button.rise(&button_cb);
                flags.wait_all(BUTTON_FLAG);
                CurrentState = RECORDKEY_STATE;
                break;
            case RECORDKEY_STATE:
                lcd.Clear(0xFFFFFFFF);
                msg_frontend(record_buff, rec_buff_len);
                sampler_ticker.attach(&ticker_cb, 20ms);
                while(index < DATA_BUFFER_SIZE){
                    flags.wait_all(TICKER_FLAG);
                    write_buf[0] = OUT_X_L | 0x80 | 0x40;
                    spi.transfer(write_buf, 7, read_buf, 7, spi_cb);
                    flags.wait_all(SPI_FLAG);
                    passkey_x[index] = (((uint16_t)read_buf[2]) << 8) | ((uint16_t)read_buf[1]);
                    passkey_y[index] = (((uint16_t)read_buf[4]) << 8) | ((uint16_t)read_buf[3]);
                    passkey_z[index] = (((uint16_t)read_buf[6]) << 8) | ((uint16_t)read_buf[5]);
                    index += 1;
                }
                sampler_ticker.detach();
                index = 0;
                lcd.Clear(0xFFFFFFFF);
                msg_frontend(record_buff_2, rec_buff_len_2);
                led_green = 1;
                button.rise(&button_cb);
                flags.wait_all(BUTTON_FLAG);
                CurrentState = ENTERKEY_STATE;
                break;
            case ENTERKEY_STATE:
                lcd.Clear(0xFFFFFFFF);
                msg_frontend(enter_buff, ent_buff_len);
                sampler_ticker.attach(&ticker_cb, 20ms);
                while(index < DATA_BUFFER_SIZE){
                    flags.wait_all(TICKER_FLAG);
                    write_buf[0] = OUT_X_L | 0x80 | 0x40;
                    spi.transfer(write_buf, 7, read_buf, 7, spi_cb);
                    flags.wait_all(SPI_FLAG);
                    attempt_x[index] = (((uint16_t)read_buf[2]) << 8) | ((uint16_t)read_buf[1]);
                    attempt_y[index] = (((uint16_t)read_buf[4]) << 8) | ((uint16_t)read_buf[3]);
                    attempt_z[index] = (((uint16_t)read_buf[6]) << 8) | ((uint16_t)read_buf[5]);
                    index += 1;
                }
                sampler_ticker.detach();
                index = 0;
                led_red = 1;
                lcd.Clear(0xFFFFFFFF);
                msg_frontend(enter_buff_2, ent_buff_len_2);
                button.rise(&button_cb);
                flags.wait_all(BUTTON_FLAG);
                CurrentState = PROCESS_STATE;
                break;
            case PROCESS_STATE:
                lcd.Clear(0xFFFFFFFF);
                passkey_dps_x[0] = FILTER_COEFFICIENT * raw_to_angVel(passkey_x[0]);
                passkey_dps_y[0] = FILTER_COEFFICIENT * raw_to_angVel(passkey_y[0]);
                passkey_dps_z[0] = FILTER_COEFFICIENT * raw_to_angVel(passkey_z[0]);

                attempt_dps_x[0] = FILTER_COEFFICIENT * raw_to_angVel(attempt_x[0]);
                attempt_dps_y[0] = FILTER_COEFFICIENT * raw_to_angVel(attempt_y[0]);
                attempt_dps_z[0] = FILTER_COEFFICIENT * raw_to_angVel(attempt_z[0]);

                for(int i = 1; i < DATA_BUFFER_SIZE; i++){
                    passkey_dps_x[i] = FILTER_COEFFICIENT * raw_to_angVel(passkey_x[i]) + (1 - FILTER_COEFFICIENT) * passkey_dps_x[i-1];
                    passkey_dps_y[i] = FILTER_COEFFICIENT * raw_to_angVel(passkey_y[i]) + (1 - FILTER_COEFFICIENT) * passkey_dps_y[i-1];
                    passkey_dps_z[i] = FILTER_COEFFICIENT * raw_to_angVel(passkey_z[i]) + (1 - FILTER_COEFFICIENT) * passkey_dps_z[i-1];

                    attempt_dps_x[i] = FILTER_COEFFICIENT * raw_to_angVel(attempt_x[i]) + (1 - FILTER_COEFFICIENT) * attempt_dps_x[i-1];
                    attempt_dps_y[i] = FILTER_COEFFICIENT * raw_to_angVel(attempt_y[i]) + (1 - FILTER_COEFFICIENT) * attempt_dps_y[i-1];
                    attempt_dps_z[i] = FILTER_COEFFICIENT * raw_to_angVel(attempt_z[i]) + (1 - FILTER_COEFFICIENT) * attempt_dps_z[i-1];
                }

                DTW(dtw_matrix, passkey_dps_x, attempt_dps_x);
                dtw_threshold_x = back_track(dtw_matrix);
                printf("done processing dtw_x: %4.5f\n", dtw_threshold_x);
                DTW(dtw_matrix, passkey_dps_y, attempt_dps_y);
                dtw_threshold_y = back_track(dtw_matrix);
                printf("done processing dtw_y: %4.5f\n", dtw_threshold_y);
                DTW(dtw_matrix, passkey_dps_z, attempt_dps_z);
                dtw_threshold_z = back_track(dtw_matrix);
                printf("done processing dtw_y: %4.5f\n", dtw_threshold_z);

                if((dtw_threshold_x>1000) | (dtw_threshold_y>1000) | (dtw_threshold_z>1000)){
                    lcd.DisplayChar(0+0*10,0,'F');
                    lcd.DisplayChar(0+1*10,0,'A');
                    lcd.DisplayChar(0+2*10,0,'I');
                    lcd.DisplayChar(0+3*10,0,'L');
                } else{
                    unlock_frontend();
                }

                button.rise(&button_cb);
                flags.wait_all(BUTTON_FLAG);
                CurrentState = IDLE_STATE;
                break;
            default:
                break;
        }
    }
}
