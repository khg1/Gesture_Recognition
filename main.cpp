// Team Member 1 : Kushal Gadhiya
// Team Member 2 : Alex Miller
// Team Member 3 : Om Sanjay Singhan

#include "mbed.h"
#include "LCD_DISCO_F429ZI.h"
#include "math.h"

// --- Register Addresses and Configuration Values ---
#define CTRL_REG1 0x20                   // Control register 1 address
#define CTRL_REG1_CONFIG 0b01'10'1'1'1'1 // Configuration: ODR=200Hz, Enable X/Y/Z axes, power on
#define CTRL_REG4 0x23                   // Control register 4 address
#define CTRL_REG4_CONFIG 0b0'0'01'0'00'0 // Configuration: High-resolution, 500dps sensitivity
#define CTRL_REG3 0x22                   // Address of Control Register 3
#define CTRL_REG3_CONFIG 0b0'0'0'0'1'000 // Enable data-ready interrupt

#define SPI_FLAG 1    // SPI communication completion flag
#define BUTTON_FLAG 2 // Event flag for user button to record passkey
#define TICKER_FLAG 3   // Event flag for sampling the data using ticker

#define DATA_BUFFER_SIZE 128    // Buffer size of the passkey and attempt key array
#define FILTER_COEFFICIENT 0.1f     //filter coefficient for the low pass filter

#define OUT_X_L 0x28       // Register address of the output data for the X axis stored on I3G4250D

#define DPS_PER_DIGIT (17.5f / 1000.0f)         //constant required for converting raw data from sensor to degress per second

//arrays to store the recorded passkey for x, y, and z axis
uint16_t passkey_x[DATA_BUFFER_SIZE], passkey_y[DATA_BUFFER_SIZE], passkey_z[DATA_BUFFER_SIZE],
    attempt_x[DATA_BUFFER_SIZE], attempt_y[DATA_BUFFER_SIZE], attempt_z[DATA_BUFFER_SIZE];

//arrays to store the registered attempt for x, y, and z axis
float passkey_dps_x[DATA_BUFFER_SIZE], passkey_dps_y[DATA_BUFFER_SIZE], passkey_dps_z[DATA_BUFFER_SIZE],
    attempt_dps_x[DATA_BUFFER_SIZE], attempt_dps_y[DATA_BUFFER_SIZE], attempt_dps_z[DATA_BUFFER_SIZE];

uint16_t dtw_matrix[DATA_BUFFER_SIZE][DATA_BUFFER_SIZE];    //cost matrix for the dynamic time warping 
float dtw_threshold_x, dtw_threshold_y, dtw_threshold_z;    //variables holding the summation of the backwards tracking

// state variable for the finite state machine
typedef enum
{
    IDLE_STATE,             //default state after reset
    RECORDKEY_STATE,        //state prompting recording of the passkey
    ENTERKEY_STATE,         //state prompting recording of the attemptkey
    PROCESS_STATE,          //state to process the raw value and compare the passkey and attemptkey for unlocking
    RETRY_STATE             //state to prompt user to retry if the attempt key does not match
} state;

EventFlags flags; // EventFlags object

InterruptIn button(BUTTON1);        //setting interrupt on user button press
DigitalOut led_red(LED2);           //for indicating the successful registration of the attempt key
DigitalOut led_green(LED1);         //for indicating the successful recording of the passkey

Ticker sampler_ticker;          //ticker to sample the data at a fixed sampling rate

LCD_DISCO_F429ZI lcd;           //initialization of lcd class

InterruptIn int2(PA_2, PullDown); // Initialize INT2 pin with pull-down resistor

//ISR for the button
void button_cb()            
{
    flags.set(BUTTON_FLAG);
}

//ISR for the SPI communication protocol
void spi_cb(int event)
{
    flags.set(SPI_FLAG);
}

//ISR for the ticker
void ticker_cb()
{
    flags.set(TICKER_FLAG);
}

//function to convert raw data from the sensor to degress per second
float raw_to_angVel(uint16_t raw_val)
{
    if ((raw_val & 0x8000) == 0x8000)
    {
        return ((float)((raw_val ^ 0xFFFF) + 1)) * DPS_PER_DIGIT * (-1);
    }
    else
    {
        return ((float)raw_val) * DPS_PER_DIGIT;
    }
}

//function to print the text messages on the LCD screen
void msg_frontend(string *lcd_buff, int buff_len)
{
    uint16_t midY = lcd.GetYSize() / 2;
    int startX = 10;
    for (int i = 0; i < buff_len; i++)
    {
        for (int j = 0; j < (int)(lcd_buff[i].length()); j++)
        {
            lcd.SetFont(&Font24);
            lcd.DisplayChar(startX + j*14, midY - (70 - i * 20), lcd_buff[i][j]);
        }
    }
}

//function to display a progressive loading bar onto the LCD indicating the fullness of the buffer array
void msg_recording(string *lcd_buff, int buff_len, int percent, int size)
{
    lcd.SetFont(&Font20);
    uint16_t midX = lcd.GetXSize() / 2;
    uint16_t midY = lcd.GetYSize() / 2;
    int startX = 10;
    for (int i = 0; i < buff_len; i++)
    {
        for (int j = 0; j < (int)(lcd_buff[i].length()); j++)
        {
            int percentage = ((percent * 100) / (size));
            char percentage_char[2];
            sprintf(percentage_char, "%d", percentage);
            if (percentage < 10)
            {
                percentage_char[1] = percentage_char[0];
                percentage_char[0] = '0';
            }
            
            lcd.DisplayChar(startX + j * 14, midY - (70 - i * 20), lcd_buff[i][j]);
            for (int k = 0; k < 2; k++)
            {
                lcd.DisplayChar(midX - (10 * (2 - k)), midY + 30, percentage_char[k]);
            }
            lcd.DisplayChar(midX + 5, midY + 30, '%');
            lcd.DrawRect(midX - size / 2, midY + 50, size, 10);
            lcd.FillRect(midX - size / 2, midY + 50, percent, 10);
        }
    }
}

//function to display the smily face with test pass message onto LCD
void unlock_frontend()
{
    // Get screen middle
    uint16_t midY = lcd.GetYSize() / 2;

    lcd.Clear(LCD_COLOR_GREEN);

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

    lcd.SetFont(&Font20);
    uint8_t testpassed[12] = "TEST PASSED";

    // Test Fail output
    lcd.SetBackColor(LCD_COLOR_GREEN);
    lcd.SetTextColor(LCD_COLOR_BLACK);
    lcd.DisplayStringAt(0,midY+midY/2, testpassed, CENTER_MODE);
    lcd.SetBackColor(LCD_COLOR_WHITE);
}

//function to display the frown face with test fail message onto LCD
void unlock_fail()
{
    // Get screen middle
    uint16_t midY = lcd.GetYSize() / 2;

    lcd.Clear(LCD_COLOR_RED);

    // Draw the face (yellow circle)
    lcd.SetTextColor(LCD_COLOR_YELLOW);
    lcd.FillCircle(120, 120, 80);

    // Draw the eyes (black circles)
    lcd.SetTextColor(LCD_COLOR_BLACK);
    lcd.FillCircle(90, 90, 10);  // Left eye
    lcd.FillCircle(150, 90, 10); // Right eye

    // Draw the smile (red arc)
    lcd.SetTextColor(LCD_COLOR_RED);
    Point frown[] = {{80, 150}, {120, 130}, {160, 150}};
    lcd.FillPolygon(frown, 3);

    // Change font size for text output
    lcd.SetFont(&Font20);

    uint8_t testfailed[12] = "TEST FAILED";

    // Test Fail output
    lcd.SetBackColor(LCD_COLOR_RED);
    lcd.SetTextColor(LCD_COLOR_BLACK);
    lcd.DisplayStringAt(0,midY+midY/2, testfailed, CENTER_MODE);
    lcd.SetBackColor(LCD_COLOR_WHITE);

}

//function to determine minimum of three numbers, required for DTW backtracking
float min_num(float num1, float num2, float num3)
{
    float min = num1;
    if (num2 < min)
    {
        min = num2;
    }
    if (num3 < min)
    {
        min = num3;
    }
    return min;
}

//function for filling dynamic time warping cost matrix based on the passkey and attemptkey
void DTW(uint16_t (*dtw_matrix)[DATA_BUFFER_SIZE], float *passkey_dps, float *attempt_dps)
{
    for (int i = 0; i < DATA_BUFFER_SIZE; i++)
    {
        for (int j = 0; j < DATA_BUFFER_SIZE; j++)
        {
            if ((i == 0) & (j == 0))
            {
                dtw_matrix[i][j] = fabsf(passkey_dps[i] - attempt_dps[j]);
            }
            else if (i == 0)
            {
                dtw_matrix[i][j] = fabsf(passkey_dps[i] - attempt_dps[j]) + dtw_matrix[i][j - 1];
            }
            else if (j == 0)
            {
                dtw_matrix[i][j] = fabsf(passkey_dps[i] - attempt_dps[j]) + dtw_matrix[i - 1][j];
            }
            else
            {
                dtw_matrix[i][j] = fabsf(passkey_dps[i] - attempt_dps[j]) + min_num(dtw_matrix[i - 1][j - 1], dtw_matrix[i - 1][j], dtw_matrix[i][j - 1]);
            }
        }
    }
}

//function for calcuating threshold values for respective axis, lower value indicates identical gestures corresponding to the axis of interest
float back_track(uint16_t (*dtw_matrix)[DATA_BUFFER_SIZE])
{
    int row_min = DATA_BUFFER_SIZE - 1;
    int col_min = DATA_BUFFER_SIZE - 1;
    int temp_row = 0, temp_col = 0;
    int count = 0;
    float cavg = 0;
    while (1)
    {
        cavg += dtw_matrix[row_min][col_min];
        float back_min = dtw_matrix[row_min][col_min - 1];
        temp_row = row_min;
        temp_col = col_min - 1;
        if (dtw_matrix[row_min - 1][col_min - 1] < back_min)
        {
            temp_row = row_min - 1;
            temp_col = col_min - 1;
            back_min = dtw_matrix[temp_row][temp_col];
        }
        if (dtw_matrix[row_min - 1][col_min] < back_min)
        {
            temp_row = row_min - 1;
            temp_col = col_min;
            back_min = dtw_matrix[temp_row][temp_col];
        }
        row_min = temp_row;
        col_min = temp_col;
        count++;
        if ((row_min == 0) | (col_min == 0))
        {
            cavg += dtw_matrix[row_min][col_min];
            count++;
            break;
        }
    }
    return (cavg / count);
}

int main()
{

    uint8_t write_buf[32], read_buf[32];                //buffer for storing data used for writing to and reading from gyroscope
    int index = 0;                                      //variable to keep track of buffer index, incremented by 1 when a new sample has been stored

    SPI spi(PF_9, PF_8, PF_7, PC_1, use_gpio_ssel);     //initializing spi class with MOSI, MISO, CLK, and CS pin along with interrupt handler

    //arrays to display messages onto LCD depending on current state
    string idle_buff[] = {"Press Button", "To Record", "Gesture"};
    int idle_buff_len = sizeof(idle_buff) / sizeof(idle_buff[0]);
    string retry_buff[] = {"Press Button", "To Try Again"};
    int retry_buff_len = sizeof(retry_buff) / sizeof(retry_buff[0]);
    string record_buff_2[] = {"PassKey", "Recorded", " ","Press Button", "To Insert", "Gesture"};
    int rec_buff_len_2 = sizeof(record_buff_2) / sizeof(record_buff_2[0]);
    string enter_buff_2[] = {"Attempt", "Registered"," ", "Press Button", "To Unlock"};
    int ent_buff_len_2 = sizeof(enter_buff_2) / sizeof(enter_buff_2[0]);

    string recording_buff[] = {"PassKey","Recording","In Progress", " ","Percent Complete"};
    int recording_buff_len = sizeof(recording_buff) / sizeof(recording_buff[0]);
    string entering_buff[] = {"Registering","Attempt","In Progress"," ","Percent Complete"};
    int entering_buff_len = sizeof(entering_buff) / sizeof(entering_buff[0]);

    spi.format(8, 3);
    spi.frequency(1'000'000);

    write_buf[0] = CTRL_REG1;
    write_buf[1] = CTRL_REG1_CONFIG;
    spi.transfer(write_buf, 2, read_buf, 2, spi_cb); // Initiate SPI transfer
    flags.wait_all(SPI_FLAG);                        // Wait until the transfer completes

    write_buf[0] = CTRL_REG4;
    write_buf[1] = CTRL_REG4_CONFIG;
    spi.transfer(write_buf, 2, read_buf, 2, spi_cb); // Initiate SPI transfer
    flags.wait_all(SPI_FLAG);                        // Wait until the transfer completes

    state CurrentState = IDLE_STATE;                //setting a default state

    while (1)
    {
        switch (CurrentState)
        {
        case IDLE_STATE:                                //Microcontroller is doing nothing and is waiting for user to press button
            lcd.Clear(0xFFFFFFFF);
            led_red = 0;
            led_green = 0;
            msg_frontend(idle_buff, idle_buff_len);
            lcd.DrawRect(150, 150, 50, 50);
            lcd.SetTextColor(LCD_COLOR_BLUE);
            lcd.FillCircle(175,175,20);
            lcd.SetTextColor(LCD_COLOR_BLACK);
            button.rise(&button_cb);
            flags.wait_all(BUTTON_FLAG);                //waits for the button to be pressed to transition to next state
            CurrentState = RECORDKEY_STATE;
            break;
        case RECORDKEY_STATE:                                       //state for recording the passkey
            lcd.Clear(0xFFFFFFFF);
            sampler_ticker.attach(&ticker_cb, 40ms);                //setting the sampling rate to 25 Hz or 40 ms, allowing user to record gestures that are 5 seconds long
            while (index < DATA_BUFFER_SIZE)                        //loop for filling the data buffer
            {
                flags.wait_all(TICKER_FLAG);                        //execution halted for 40ms
                write_buf[0] = OUT_X_L | 0x80 | 0x40;
                spi.transfer(write_buf, 7, read_buf, 7, spi_cb);            //requests the data for the x, y, and z axis from the sensor
                flags.wait_all(SPI_FLAG);
                passkey_x[index] = (((uint16_t)read_buf[2]) << 8) | ((uint16_t)read_buf[1]); 
                passkey_y[index] = (((uint16_t)read_buf[4]) << 8) | ((uint16_t)read_buf[3]);
                passkey_z[index] = (((uint16_t)read_buf[6]) << 8) | ((uint16_t)read_buf[5]);
                index += 1;

                msg_recording(recording_buff, recording_buff_len, index, DATA_BUFFER_SIZE);         // For displaying recording percentage
            }
            sampler_ticker.detach();                            //disabling the ticker once the buffer is full
            index = 0;
            lcd.Clear(0xFFFFFFFF);
            msg_frontend(record_buff_2, rec_buff_len_2);        //message indicating the pass key has been recorded
            lcd.DrawRect(150, 175, 50, 50);
            lcd.SetTextColor(LCD_COLOR_BLUE);
            lcd.FillCircle(175,200,20);
            lcd.SetTextColor(LCD_COLOR_BLACK);
            led_green = 1;                                      //visual feedback using green led for successful completion of the recording stage
            button.rise(&button_cb);
            flags.wait_all(BUTTON_FLAG);
            CurrentState = ENTERKEY_STATE;
            break;
        case ENTERKEY_STATE:                        //state for recording the attemptkey
            lcd.Clear(0xFFFFFFFF);
            sampler_ticker.attach(&ticker_cb, 40ms);
            while (index < DATA_BUFFER_SIZE)
            {
                flags.wait_all(TICKER_FLAG);
                write_buf[0] = OUT_X_L | 0x80 | 0x40;
                spi.transfer(write_buf, 7, read_buf, 7, spi_cb);
                flags.wait_all(SPI_FLAG);
                attempt_x[index] = (((uint16_t)read_buf[2]) << 8) | ((uint16_t)read_buf[1]);
                attempt_y[index] = (((uint16_t)read_buf[4]) << 8) | ((uint16_t)read_buf[3]);
                attempt_z[index] = (((uint16_t)read_buf[6]) << 8) | ((uint16_t)read_buf[5]);
                index += 1;

                msg_recording(entering_buff, entering_buff_len, index, DATA_BUFFER_SIZE);       // For displaying recording percentage
            }
            sampler_ticker.detach();
            index = 0;
            led_red = 1;
            lcd.Clear(0xFFFFFFFF);
            msg_frontend(enter_buff_2, ent_buff_len_2);
            lcd.DrawRect(150, 175, 50, 50);
            lcd.SetTextColor(LCD_COLOR_BLUE);
            lcd.FillCircle(175,200,20);
            lcd.SetTextColor(LCD_COLOR_BLACK);
            button.rise(&button_cb);
            flags.wait_all(BUTTON_FLAG);
            CurrentState = PROCESS_STATE;
            break;
        case PROCESS_STATE:                             //state for filtering the acquired data, running DTW algorithm, and displaying if device is unlocked or locked
            lcd.Clear(0xFFFFFFFF);

            passkey_dps_x[0] = FILTER_COEFFICIENT * raw_to_angVel(passkey_x[0]);
            passkey_dps_y[0] = FILTER_COEFFICIENT * raw_to_angVel(passkey_y[0]);
            passkey_dps_z[0] = FILTER_COEFFICIENT * raw_to_angVel(passkey_z[0]);

            attempt_dps_x[0] = FILTER_COEFFICIENT * raw_to_angVel(attempt_x[0]);
            attempt_dps_y[0] = FILTER_COEFFICIENT * raw_to_angVel(attempt_y[0]);
            attempt_dps_z[0] = FILTER_COEFFICIENT * raw_to_angVel(attempt_z[0]);

            for (int i = 1; i < DATA_BUFFER_SIZE; i++)                                          // low pass filter
            {
                passkey_dps_x[i] = FILTER_COEFFICIENT * raw_to_angVel(passkey_x[i]) + (1 - FILTER_COEFFICIENT) * passkey_dps_x[i - 1];
                passkey_dps_y[i] = FILTER_COEFFICIENT * raw_to_angVel(passkey_y[i]) + (1 - FILTER_COEFFICIENT) * passkey_dps_y[i - 1];
                passkey_dps_z[i] = FILTER_COEFFICIENT * raw_to_angVel(passkey_z[i]) + (1 - FILTER_COEFFICIENT) * passkey_dps_z[i - 1];

                attempt_dps_x[i] = FILTER_COEFFICIENT * raw_to_angVel(attempt_x[i]) + (1 - FILTER_COEFFICIENT) * attempt_dps_x[i - 1];
                attempt_dps_y[i] = FILTER_COEFFICIENT * raw_to_angVel(attempt_y[i]) + (1 - FILTER_COEFFICIENT) * attempt_dps_y[i - 1];
                attempt_dps_z[i] = FILTER_COEFFICIENT * raw_to_angVel(attempt_z[i]) + (1 - FILTER_COEFFICIENT) * attempt_dps_z[i - 1];
            }

            DTW(dtw_matrix, passkey_dps_x, attempt_dps_x);                  //cost matrix for the x axis
            dtw_threshold_x = back_track(dtw_matrix);                       //threshold for x axis
            printf("done processing dtw_x: %4.5f\n", dtw_threshold_x);
            DTW(dtw_matrix, passkey_dps_y, attempt_dps_y);                  //cost matrix for the y axis
            dtw_threshold_y = back_track(dtw_matrix);                       //threshold for y axis
            printf("done processing dtw_y: %4.5f\n", dtw_threshold_y);
            DTW(dtw_matrix, passkey_dps_z, attempt_dps_z);                  //cost matrix for the z axis
            dtw_threshold_z = back_track(dtw_matrix);                       //threshold for z axis
            printf("done processing dtw_y: %4.5f\n", dtw_threshold_z);

            if ((dtw_threshold_x > 1000) | (dtw_threshold_y > 1000) | (dtw_threshold_z > 1000))      //condition for determining the output, unlock if threshold value of axis are lower than 1000
            {
                unlock_fail();                              //still locked
                CurrentState = RETRY_STATE;                 //next state to retry
            }
            else
            {
                unlock_frontend();                          //unlocked
                CurrentState = IDLE_STATE;                  //next state to idle
            }

            button.rise(&button_cb);
            flags.wait_all(BUTTON_FLAG);
            break;
        case RETRY_STATE:                                   //state for allowing user to re-entre the attempt response again in case of failure
            lcd.Clear(0xFFFFFFFF);
            led_red = 0;
            msg_frontend(retry_buff, retry_buff_len);
            lcd.DrawRect(150, 150, 50, 50);
            lcd.SetTextColor(LCD_COLOR_BLUE);
            lcd.FillCircle(175,175,20);
            lcd.SetTextColor(LCD_COLOR_BLACK);
            button.rise(&button_cb);
            flags.wait_all(BUTTON_FLAG);
            CurrentState = ENTERKEY_STATE;
            break;
        default:
            break;
        }
    }
}