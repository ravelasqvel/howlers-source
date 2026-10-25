#include "tasks.h"

void create_tasks(void)
{   
    ///< Create a task wifi_task to manage the Wi-Fi connection and UDP server maximum priority
    xTaskCreate(app_network_task, "wifi_task", 10*1024, NULL, 24, NULL);
    
    ///< Create a task to handle the UART events
    xTaskCreate(uart_event_task, "uart_event_task", 4*1024, NULL, 20, NULL);

    ///< Create a task to manage the BNO055 sensor
    xTaskCreate(bno055_task, "bno055_task", 4*1024, NULL, 9, &gSys.task_handle_bno055);

    ///< Create the control task
    xTaskCreate(bldc_control_task, "bldc_control_task", 6*1024, NULL, 12, &gSys.task_handle_bldc_ctrl);

    ///< Create the trigger task
    xTaskCreate(robot_control_task, "robot_control_task", 6*1024, NULL, 11, &gSys.task_handle_robot_ctrl);

    ///< Create the save task
    //xTaskCreate(save_data_task, "save_nvs_task", 60*1024, NULL, 2, &gSys.task_handle_save);

}



bool create_kernel_objects(void)
{    
    gSys.queue = xQueueCreate(10, sizeof(uint8_t)*45); ///< Create a queue to send the data to the save task
    if (gSys.queue == NULL) {
        ESP_LOGI("init_system", "Queue not created");
        return false;
    }

    gSys.mutex = xSemaphoreCreateMutex(); ///< Create a mutex to protect the access to the global variables
    if (gSys.mutex == NULL) {
        ESP_LOGI("init_system", "Mutex not created");
        return false;
    }
    
    gSys.mtx_printf = xSemaphoreCreateMutex(); ///< Create a mutex to protect the access to the printf function
    if (gSys.mtx_printf == NULL) {
        ESP_LOGI("init_system", "Mutex for printf not created");
        return false;
    }

    gSys.smph_bldc = xSemaphoreCreateBinary(); ///< Create a semaphore to synchronize the motor identification task
    if (gSys.smph_bldc == NULL) {
        ESP_LOGI("init_system", "Semaphore for BLDC not created");
        return false;
    }
    gSys.mtx_cntrl = xSemaphoreCreateMutex(); ///< Create a mutex to protect the access to the control variables
    if (gSys.mtx_cntrl == NULL) {
        ESP_LOGI("init_system", "Mutex for control not created");
        return false;
    }
    gSys.mtx_traj = xSemaphoreCreateMutex(); ///< Create a mutex to protect the access to the trajectory variables
    if (gSys.mtx_traj == NULL) {
        ESP_LOGI("init_system", "Mutex for trajectory not created");
        return false;
    }
    gSys.queue_bldc = xQueueCreate(1, sizeof(float) * 3);
    if (gSys.queue_bldc == NULL) {
        ESP_LOGI("init_system", "Queue for BLDC not created");
        return false;
    }
    gSys.queue_robot = xQueueCreate(1, sizeof(float) * 3);
    if (gSys.queue_robot == NULL) {
        ESP_LOGI("init_system", "Queue for robot not created");
        return false;
    }
    gSys.queue_wifi = xQueueCreate(1, sizeof(float) * 3);
    if (gSys.queue_wifi == NULL) {
        ESP_LOGI("init_system", "Queue for WiFi not created");
        return false;
    }
    gSys.queue_bno055 = xQueueCreate(1, sizeof(float) * 3);
    if (gSys.queue_bno055 == NULL) {
        ESP_LOGI("init_system", "Queue for BNO055 not created");
        return false;
    }
    return true; 
}

void bno055_task(void *pvParameters)
{
    float acc[3] = {0}; ///< Accelerometer data: acc[0] = ax, acc[1] = ay, acc[2] = omega (rad/s)
    float acc_prev[3] = {0};
    float yaw_prev = 0;
    float delta = 0;

    ///< Create kalman filter for the BNO055 sensor
    kalman1D_t kf_bno055[3];
    for (int i = 0; i < 3; i++) {
        kalman1D_init(&kf_bno055[i], KALMAN_1D_BNO055_Q, KALMAN_1D_BNO055_R*2);
    }

    while (true) {
        ///< Wait for the notification from the timer
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
      
        ///< Read All data from BNO055 sensor
        if (BNO055_ReadAll_Lineal(&gBNO055)) { // error reading data 
            acc[0] = acc_prev[0];
            acc[1] = acc_prev[1];
            acc[2] = acc_prev[2];

        }else {
            float delta = gBNO055.yaw - yaw_prev;
            if      (delta >  M_PI) delta -= 2 * M_PI;
            else if (delta < -M_PI) delta += 2 * M_PI;

            ///< Data from the BNO055 sensor
            acc[0] = kalman1D_update(&kf_bno055[0], gBNO055.ax);
            acc[1] = kalman1D_update(&kf_bno055[1], gBNO055.ay);
            acc[2] = kalman1D_update(&kf_bno055[2], delta * SAMPLING_RATE_HZ); 

            ///< Update
            yaw_prev = gBNO055.yaw;
            acc_prev[0] = acc[0];
            acc_prev[1] = acc[1];
            acc_prev[2] = acc[2];
        }

        ///< Send the data to the queue for the control task
        xQueueOverwrite(gSys.queue_bno055, (void *)acc); 
    }
    vTaskDelete(NULL);
}

void robot_control_task(void *pvParameters)
{
    float vb[3] = {0, 0, 0}; ///< Body velocities: vb[0] = vbx, vb[1] = vby, vb[2] = wb
    float vb_sp[3] = {0}; ///< Body velocity setpoints.
    float vb_sp_prev[3] = {0}; ///< Previous body velocity setpoints.
    float omega[3] = {0}; ///< Angular velocities: omega[0] = w1, omega[1] = w2, omega[2] = w3
    float w_sp[3] = {0}; ///< Angular velocity setpoints: w_sp[0] = w1_sp, w_sp[1] = w2_sp, w_sp[2] = w3_sp
    float acc[3] = {0}; ///< Accelerometer data: acc[0] = ax, acc[1] = ay, acc[2] = omega (rad/s)
    float pid_output[3] = {0}; ///< PID output body velocities
    int cnt = 0;

    ///< Initialize the sensor fusion filter: fusion[0] for vbx, fusion[1] for vby
    senfusion_t fusion[3];
    senfusion_config_t cfg = {
        .A  = 1.0f,        // v_k = v_{k-1} + u
        .B  = 1.0f,        // u = a·dt
        .Q  = SENFUSION_1D_Q,  // process noise var
        .R1 = SENFUSION_1D_R1, // body speed var (m/s)^2
        .R2 = SENFUSION_1D_R2, // IMU var (rad/s)^2
    };

    senfusion_init(&fusion[0], &cfg);
    senfusion_init(&fusion[1], &cfg);
    senfusion_init(&fusion[2], &cfg);
    fusion[2].B = 0.0f; // No input for the omega velocity

    ///< Initialize controllers for body velocities
    pid_block_t config_pid = {
        .Kp = 2.1167, ///< Proportional gain
        .Ki = 25.0639, ///< Integral gain
        .Kd = 0, ///< Derivative gain
        .max_output   = 60,
        .min_output   = -60,
        .max_integral = 200,
        .min_integral = -200,
        .a1 = 0.5, .a0 = -0.4985, .b1 = 1, .b0 = -1,
        .order = 1,
    };
    control_t vb_ctrl[3];
    for (int i = 0; i < 3; i++) {
        control_init(&vb_ctrl[i], 1.0f / SAMPLING_RATE_HZ, config_pid);
    }

    while (true) {
        ///< Receive the accelerometer data from the BNO055 task
        xQueueReceive(gSys.queue_bno055, (void *)acc, portMAX_DELAY);

        ///< Get wheels angular velocities and calculate body velocities with forward kinematics
        xQueueReceive(gSys.queue_robot, (void *)omega, 0);
        calc_fordwkinematics(omega[0], omega[1], omega[2], &vb[0], &vb[1], &vb[2]);

        ///< Receive the body velocities setpoints from the queue
        BaseType_t status = xQueueReceive(gSys.queue_wifi, (void *)vb_sp, 0);
        if (status != pdTRUE) { ///< If there is no data in the queue, use default values
            vb_sp[0] = vb_sp_prev[0];
            vb_sp[1] = vb_sp_prev[1];
            vb_sp[2] = vb_sp_prev[2];
        }
        vb_sp_prev[0] = vb_sp[0];
        vb_sp_prev[1] = vb_sp[1];
        vb_sp_prev[2] = vb_sp[2];

        ///< Apply sensor fusion to the body velocities
        for (int i = 0; i < 3; i++) {
            senfusion_predict(&fusion[i], acc[i] / SAMPLING_RATE_HZ);
            senfusion_update1(&fusion[i], vb[i]);
            if (i == 2) {
                senfusion_update2(&fusion[i], acc[i]);
            }
        }

        ///< PI control for body velocities
        for (int i = 0; i < 3; i++) {
            control_set_setpoint(&vb_ctrl[i], vb_sp[i]); ///< Set the setpoint for each body velocity
            pid_output[i] = control_calc_pid_z(&vb_ctrl[i], senfusion_get_state(&fusion[i]));
        }

        ///< Inverse kinematics to calculate the setpoints for each motor
        calc_invkinematics(pid_output[0], pid_output[1], pid_output[2], &w_sp[0], &w_sp[1], &w_sp[2]);
        xQueueOverwrite(gSys.queue_bldc, (void *)w_sp);

        // ///< Print states
        // wrap_printf("A, %d, %.4f, %.4f, %.4f \r\n", // euler
        //             cnt*SAMPLING_RATE_HZ,
        //             vb[0], vb[1], vb[2]);
        // wrap_printf("I, %d, %.4f, %.4f, %.4f, %.4f, %.4f, %.4f\r\n", // gyros, acce
        //             cnt*SAMPLING_RATE_HZ,
        //             acc[0], acc[1], acc[2],
        //             senfusion_get_state(&fusion[0]), senfusion_get_state(&fusion[1]), senfusion_get_state(&fusion[2]));
        // wrap_printf("H, %d, %.4f, %.4f, %.4f \r\n",  // high-g
        //             cnt*SAMPLING_RATE_HZ,
        //             pid_output[0], pid_output[1], pid_output[2]);
        // wrap_printf("M, %d, %.4f, %.4f, %.4f \r\n", // magne
        //             cnt*SAMPLING_RATE_HZ,
        //             w_sp[0], w_sp[1], w_sp[2]);
        cnt++;

    }
    vTaskDelete(NULL);
}

void bldc_control_task(void *pvParameters)
{
    ///< Timer wich will be used for all motors
    const esp_timer_create_args_t timer_args = {
        .callback = &motor_bldc_cb,
        .arg = NULL,
        .name = "bldc-ident"
    };
    esp_timer_handle_t timer_handle;
    ESP_ERROR_CHECK(esp_timer_create(&timer_args, &timer_handle));
    gSys.timer_bldc = timer_handle;
    ESP_ERROR_CHECK(esp_timer_start_periodic(gSys.timer_bldc, TIME_SAMP_MOTOR_US));

    ///< Initialize the PID controllers for each motor
    control_set_setpoint(&gCtrl[0], 0);
    control_set_setpoint(&gCtrl[1], 0);
    control_set_setpoint(&gCtrl[2], 0);

    ///< Initialize some variables for the control task
    float duty[3] = {0};
    float omega[3] = {0, 0, 0}; // in rad/s
    float w_sp[3] = {0}; // angle speed setpoints in rad/s
    float w_sp_prev[3] = {0,0,0}; // in rad/s
    uint32_t cnt = 0;

    ///< Kalman filter variables for encoders
    kalman1D_t kalman_enc[3];
    for (int i = 0; i < 3; i++) {
        kalman1D_init(&kalman_enc[i], KALMAN_1D_ENC_Q, KALMAN_1D_ENC_R);
    }

    while (true) {
        xSemaphoreTake(gSys.smph_bldc, portMAX_DELAY);

        ///< Control variables: get angle and calculate speed
        for (int i = 0; i < 3; i++) {
            omega[i] = calculate_motor_speed(&kalman_enc[i], i);
        }
        ///< Send the angular velocities to the robot control task: 
        ///< overwrite the queue so that the last value is always the current one
        xQueueOverwrite(gSys.queue_robot, (void *)omega);

        ///< Get the current setpoints for each motor
        BaseType_t status = xQueueReceive(gSys.queue_bldc, (void *)w_sp, 0); ///< Receive the setpoints from the queue
        if (status != pdTRUE) { ///< If there is no data in the queue, use default values
            w_sp[0] = w_sp_prev[0];
            w_sp[1] = w_sp_prev[1];
            w_sp[2] = w_sp_prev[2];
        }
        w_sp_prev[0] = w_sp[0];
        w_sp_prev[1] = w_sp[1];
        w_sp_prev[2] = w_sp[2];

        ///< Control: calculate the duty cycle for each motor using the PID controller
        xSemaphoreTake(gSys.mtx_cntrl, portMAX_DELAY); 
        for (int i = 0; i < 3; i++) {
            control_set_setpoint(&gCtrl[i], w_sp[i]); ///< Set the setpoint for each motor
            duty[i] = control_calc_pid_z(&gCtrl[i], omega[i]);
        }
        xSemaphoreGive(gSys.mtx_cntrl); ///< Give the mutex to protect the access to the control variables

        ///< Set the duty cycle of the motors
        for (int i = 0; i < 3; i++) {
            bldc_set_duty_motor(&gMotor[i], duty[i]);
        }

        // ///< Send the data via serial
        // if (cnt%10 == 0){
        //     wrap_printf("I,%d,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\r\n", cnt*SAMP_RATE_MOTOR_HZ,
        //                 duty[0], duty[1], duty[2], omega[0], omega[1], omega[2]);
        // }
        cnt++; ///< Increment the counter
    }
    vTaskDelete(NULL);
}

void save_data_task(void *pvParameters)
{
    ///< Label to print at the beginning of the file
    char label[] = "Duty\tAngle(deg)\tAcce(m^2)\tDist(m)\tPos(m)\tVel(m/s)\n";
    label[strlen(label)] = '\0'; ///< Add the null terminator to the string

    ///< Buffer to save the data from the queue
    uint8_t data[45]; 

    ///< Buffer to save the data from the sensors. Each sample has 6 values: duty, angle, acceleration, distance, position, velocity.
    int8_t buffer[10*SAMPLING_RATE_HZ + 1][45]; // each sample sizes 40 bytes, for 10 seconds of data

    while (true) {
        ///< Wait for the data from the control task
        xQueueReceive(gSys.queue, (void *const)data, portMAX_DELAY); ///< Receive the data from the queue
        uint8_t length = strlen((const char *)data); ///< Get the length of the data

        ///< Save the data to the buffer
        data[length] = '\0'; ///< Add the null terminator to the data
        strncpy((char *)buffer[gSys.cnt_sample], (const char *)data, length + 1); ///< Copy the data to the buffer

        ///< If the buffer is full
        if (gSys.cnt_sample >= 10*SAMPLING_RATE_HZ) {
            ESP_LOGI("save_data_task", "Buffer full, printing data to console");

            ///< Print the data to the console
            printf("%s", label); ///< Print the label to the console
            for (int i = 0; i < gSys.cnt_sample; i++) {
                printf("%s", buffer[i]); ///< Print the data to the console
                if (i % 200 == 0) vTaskDelay(pdMS_TO_TICKS(100)); ///< Delay to avoid blocking the task for too long
            }

            printf("Total samples: %d\n", gSys.cnt_sample); ///< Print the total number of samples
            printf("Data saved successfully.\n");

            ///< Reset the buffer
            memset(buffer, 0, sizeof(buffer)); ///< Clear the buffer

            gSys.cnt_sample = 0; ///< Reset the number of samples
        }
    }
    vTaskDelete(NULL);
}


void uart_event_task(void *pvParameters)
{
    uart_event_t event;

    while(true) {
        if(xQueueReceive(gUc.uart_queue, (void * )&event, (TickType_t)portMAX_DELAY)) {
            switch(event.type) {
            case UART_DATA: ///< Event of UART receving data
                ESP_LOGI(TAG_UART_TASK, "UART_DATA");
                uconsole_read_data(&gUc);

                parse_and_update_setpoints((const char *)gUc.data);
                break;

            case UART_FIFO_OVF: ///< Event of HW FIFO overflow
                ESP_LOGI(TAG_UART_TASK, "UART_FIFO_OVF");
                uart_flush_input(gUc.uart_num);
                xQueueReset(gUc.uart_queue);
                break;

            case UART_BUFFER_FULL: ///< Event of UART ring buffer full
                ESP_LOGI(TAG_UART_TASK, "UART_BUFFER_FULL");
                uart_flush_input(gUc.uart_num);
                xQueueReset(gUc.uart_queue);
                break;

            default:
                ESP_LOGI(TAG_UART_TASK, "uart event type: %d", event.type);
                break;
            }
        }
    }
    vTaskDelete(NULL);
}

void app_network_task(void *pvParameters) {
    wifi_prepare();

    if (!wifi_sta_init(WIFI_SSID, WIFI_PASS, &gIpAddr)) {
        ESP_LOGE("APP", "Wi-Fi connection failed");
        vTaskDelete(NULL);
    }

    ESP_LOGI("APP", "Wi-Fi connected successfully");
    ESP_LOGI("APP", "IP Address: " IPSTR, IP2STR(&gIpAddr));

    // ==== Comienza el servidor UDP directamente aquí ====
    char rx_buffer[UDP_RX_BUF_SIZE];
    struct sockaddr_in listen_addr = {
        .sin_addr.s_addr = htonl(INADDR_ANY),
        .sin_family = AF_INET,
        .sin_port = htons(UDP_PORT)
    };

    int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
    if (sock < 0) {
        ESP_LOGE("UDP", "Unable to create socket: errno %d", errno);
        vTaskDelete(NULL);
    }
    ESP_LOGI("UDP", "Socket created");

    if (bind(sock, (struct sockaddr *)&listen_addr, sizeof(listen_addr)) < 0) {
        ESP_LOGE("UDP", "Socket bind failed: errno %d", errno);
        close(sock);
        vTaskDelete(NULL);
    }
    ESP_LOGI("UDP", "Socket bound on port %d", UDP_PORT);

    while (true) {
        
        //// TESTING: Send a test message every second
        // vTaskDelay(pdMS_TO_TICKS(1000));
        // float vb_sp[3] = {0, 0.3, 0};
        // xQueueOverwrite(gSys.queue_wifi, (void *)vb_sp); ///< Overwrite the queue with the setpoints

        struct sockaddr_in source_addr;
        socklen_t socklen = sizeof(source_addr);
        int len = recvfrom(sock, rx_buffer, sizeof(rx_buffer) - 1, 0,
                           (struct sockaddr *)&source_addr, &socklen);

        if (len < 0) {
            ESP_LOGE("UDP", "recvfrom failed: errno %d", errno);
            break;
        }

        rx_buffer[len] = '\0'; // Null-terminate
        ESP_LOGI("UDP", "Received %d bytes from %s: '%s'",
                 len, inet_ntoa(source_addr.sin_addr), rx_buffer);

        // parse_command(rx_buffer, len); // Process the received command
    }

    close(sock);
    vTaskDelete(NULL);
}
