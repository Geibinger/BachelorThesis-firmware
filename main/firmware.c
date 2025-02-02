#include <string.h>
#include <stdio.h>
#include <unistd.h>
#include <stdbool.h>
#include <stdlib.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_system.h"

#include "nvs_flash.h"
#include "nvs.h"

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rclc_parameter/rclc_parameter.h>

#include <rmw_microros/rmw_microros.h>
#include <uxr/client/config.h>

#include <type_utilities.h>
#include <tuw_nav_msgs/msg/joints_iws.h> // Custom message type (defined in components/micro_ros_espidf_component/extra_packages/tuw_msgs/tuw_nav_msgs/msg/JointsIWS.msg)

#ifdef RMW_UXRCE_TRANSPORT_CUSTOM
#include "esp32_serial_transport.h" // User-provided custom transport
#elif defined(RMW_UXRCE_TRANSPORT_UDP)
#include <uros_network_interfaces.h>
#endif

#define TAG "PARAM_NODE"

#define RCCHECK(fn) do { \
    rcl_ret_t temp_rc = fn; \
    if ((temp_rc != RCL_RET_OK)) { \
        ESP_LOGE(TAG, "Failed status on line %d: %d. Aborting.", __LINE__, (int)temp_rc); \
        vTaskDelete(NULL); \
    } \
} while(0)

#define RCSOFTCHECK(fn) do { \
    rcl_ret_t temp_rc = fn; \
    if ((temp_rc != RCL_RET_OK)) { \
        ESP_LOGW(TAG, "Soft fail status on line %d: %d. Continuing.", __LINE__, (int)temp_rc); \
    } \
} while(0)


// TODO: implement use_sim_time parameter. If set to true -> use subscriber to /clock topic to get time for messages. If false -> use system time with https://micro.ros.org/docs/tutorials/programming_rcl_rclc/micro-ROS/#time-sync
// TODO: Improve structure -> what would be a modular, yet efficient way to implement this?

rclc_parameter_server_t param_server;
rcl_publisher_t publisher;
tuw_nav_msgs__msg__JointsIWS msg;
// Defined as:
// # Message for sensing/control of an indepenend N wheel steering platform
// std_msgs/Header header

// #the type defines the usage and units such as cmd_torque, cmd_acceleration, cmd_velocity, cmd_position, measured_torque, measured_acceleration, measured_velocity, measured_position
// string type_steering
// string type_revolute 

// #actual sensing/control variables of the defined type
// float64[] steering
// float64[] revolute

// Forward declarations
static void load_param_from_nvs(rclc_parameter_server_t *server);
static esp_err_t save_param_to_nvs(const rcl_interfaces__msg__Parameter * new_param);

void timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{
	RCLC_UNUSED(last_call_time);
	if (timer != NULL) {
        // Increment the first element of the 'steering' array
        msg.steering.data[0]++;
        ESP_LOGI(TAG, "Publishing: %d", (int) msg.steering.data[0]);
		RCSOFTCHECK(rcl_publish(&publisher, &msg, NULL));
	}
}

/**
 * @brief Callback triggered whenever a parameter is changed via ROS 2.
 *
 * Prints both old and new values to diagnose unexpected overwrites.
 *
 * @param old_param The previous value of the parameter.
 * @param new_param The new value of the parameter.
 * @param context   User-defined context (unused in this example).
 * @return          true if the callback handled the change successfully.
 */
bool on_parameter_changed_cb(
    const rcl_interfaces__msg__Parameter * old_param,
    const rcl_interfaces__msg__Parameter * new_param,
    void * context)
{
    (void)context; // Not used in this example

    // Print old vs new for debugging
    ESP_LOGI(TAG, "Parameter '%s' modified.", new_param->name.data);

    // Print old value
    if (old_param) {
        ESP_LOGI(TAG, "Old value type: %d", (int)old_param->value.type);
        switch (old_param->value.type) {
            case RCLC_PARAMETER_INT:
                ESP_LOGI(TAG, "Old value: %lld (int64_t)", (long long)old_param->value.integer_value);
                break;
            case RCLC_PARAMETER_BOOL:
                ESP_LOGI(TAG, "Old value: %d (bool)", old_param->value.bool_value);
                break;
            case RCLC_PARAMETER_DOUBLE:
                ESP_LOGI(TAG, "Old value: %f (double)", old_param->value.double_value);
                break;
            default:
                ESP_LOGI(TAG, "Old value: unknown type");
                break;
        }
    } else {
        ESP_LOGI(TAG, "Old value is NULL (no previous value?)");
    }

    // Print new value
    switch (new_param->value.type) {
        case RCLC_PARAMETER_BOOL:
            ESP_LOGI(TAG, "New value: %d (bool)", new_param->value.bool_value);
            break;
        case RCLC_PARAMETER_INT:
            ESP_LOGI(TAG, "New value: %lld (int64_t)", (long long)new_param->value.integer_value);
            break;
        case RCLC_PARAMETER_DOUBLE:
            ESP_LOGI(TAG, "New value: %f (double)", new_param->value.double_value);
            break;
        default:
            ESP_LOGW(TAG, "Unknown parameter type.");
            break;
    }

    // Persist to NVS
    esp_err_t err = save_param_to_nvs(new_param);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to persist parameter to NVS (%d)", err);
    }

    return true; // Accept the new parameter value
}

/**
 * @brief micro_ros_task: initializes the node, parameter server, and loads param from NVS.
 */
void micro_ros_task(void * arg)
{
    ESP_UNUSED(arg);

    rcl_ret_t rc;
    rcl_allocator_t allocator = rcl_get_default_allocator();
    rclc_support_t support;

    // Initialize the default init_options
    rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
    RCCHECK(rcl_init_options_init(&init_options, allocator));

#ifdef RMW_UXRCE_TRANSPORT_CUSTOM
    // Custom serial transport
    size_t uart_port = UART_NUM_1; // Example UART port
    RCCHECK(rmw_uros_set_custom_transport(
        true,
        (void *)&uart_port,
        esp32_serial_open,
        esp32_serial_close,
        esp32_serial_write,
        esp32_serial_read
    ));
#elif defined(RMW_UXRCE_TRANSPORT_UDP)
    // Set UDP address and port
    rmw_init_options_t* rmw_options = rcl_init_options_get_rmw_init_options(&init_options);
    RCCHECK(rmw_uros_options_set_udp_address(CONFIG_MICRO_ROS_AGENT_IP, CONFIG_MICRO_ROS_AGENT_PORT, rmw_options));
#endif

    // Create the RCLC support structure
    RCCHECK(rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator));

    // Create ROS node
    rcl_node_t node = rcl_get_zero_initialized_node();
    RCCHECK(rclc_node_init_default(&node, "esp32_param_node", "", &support));

    // Create publisher
	RCCHECK(rclc_publisher_init_default(
		&publisher,
		&node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(tuw_nav_msgs, msg, JointsIWS),
        "joints_iws_test"
    ));

	// create timer,
	rcl_timer_t timer;
	const unsigned int timer_timeout = 1000;
	RCCHECK(rclc_timer_init_default2(
		&timer,
		&support,
		RCL_MS_TO_NS(timer_timeout),
		timer_callback,
        true
    ));

    // Initialize parameter server
    RCCHECK(rclc_parameter_server_init_default(&param_server, &node));

    // Initialize executor with enough handles
    rclc_executor_t executor = rclc_executor_get_zero_initialized_executor();
    RCCHECK(rclc_executor_init(&executor, &support.context, RCLC_EXECUTOR_PARAMETER_SERVER_HANDLES + 1, &allocator));

    // Add parameter server to executor with the callback
    RCCHECK(rclc_executor_add_parameter_server(&executor, &param_server, on_parameter_changed_cb));

    // Add timer to executor
	RCCHECK(rclc_executor_add_timer(&executor, &timer));

    // Declare parameter 'param1' of type INT (NO default set here)
    RCCHECK(rclc_add_parameter(&param_server, "param1", RCLC_PARAMETER_INT));

    // Load the parameter from NVS. If not found, set it to 10.
    load_param_from_nvs(&param_server);

    // Retrieve parameter value and log
    int64_t param1_val;
    if (rclc_parameter_get_int(&param_server, "param1", &param1_val) == RCL_RET_OK)
    {
        ESP_LOGI(TAG, "Initial value of param1 (post-NVS load): %lld", (long long)param1_val);
    }
    else
    {
        // If param1 wasn't found in NVS and not set, let's initialize it to 10
        ESP_LOGW(TAG, "param1 not set, defaulting to 10 now.");
        RCCHECK(rclc_parameter_set_int(&param_server, "param1", 10));
    }


    // TODO: Instead of manually setting the message, use <type_utilities.h> (https://micro.ros.org/docs/tutorials/advanced/handling_type_memory/)
    // Define the message to publish
    msg.header.frame_id.capacity = 20;
    msg.header.frame_id.data = (char*) malloc(msg.header.frame_id.capacity * sizeof(char));
    msg.header.frame_id.size = 0;

    // Assigning value to the frame_id char sequence
    strcpy(msg.header.frame_id.data, "test_frame_id");
    msg.header.frame_id.size = strlen(msg.header.frame_id.data);

    // Assigning value to other members
    msg.header.stamp.sec = 10; // TODO: Sync with system time (https://micro.ros.org/docs/tutorials/programming_rcl_rclc/micro-ROS/#time-sync)
    msg.header.stamp.nanosec = 20;

    msg.type_steering.capacity = 20;
    msg.type_steering.data = (char*) malloc(msg.type_steering.capacity * sizeof(char));
    msg.type_steering.size = 0;
    strcpy(msg.type_steering.data, "cmd_position");
    msg.type_steering.size = strlen(msg.type_steering.data);

    msg.type_revolute.capacity = 20;
    msg.type_revolute.data = (char*) malloc(msg.type_revolute.capacity * sizeof(char));
    msg.type_revolute.size = 0;
    strcpy(msg.type_revolute.data, "cmd_velocity");
    msg.type_revolute.size = strlen(msg.type_revolute.data);

    static double steering_data[2] = {0.0, 0.0}; // ! float64 -> double
    msg.revolute.capacity = 2; // Maximum number of elements
    msg.revolute.data = steering_data;
    msg.revolute.size = 2; // Number of elements in use (must be <= capacity) -> in this example we will have one element that increments and one that stays at 0

    static double revolute_data[2] = {0.0, 0.0};
    msg.steering.capacity = 2; // Maximum number of elements
    msg.steering.data = revolute_data;
    msg.steering.size = 2; // Number of elements in use (must be <= capacity) -> in this example we will have one element that increments and one that stays at 0

    // Spin the executor
    while (1)
    {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
        vTaskDelay(pdMS_TO_TICKS(100)); // Sleep for 100 ms
    }

    // Clean up (Unreachable in this example due to infinite loop)
    rc = rclc_executor_fini(&executor);
    rc += rclc_parameter_server_fini(&param_server, &node);
    rc += rcl_node_fini(&node);

    if (rc != RCL_RET_OK)
    {
        ESP_LOGE(TAG, "Error while cleaning up!");
    }

    vTaskDelete(NULL);
}

/**
 * @brief Attempts to load 'param1' from NVS. If found, sets the parameter server value.
 *
 * Does not set any default here; that is left for after this function call if not found.
 */
static void load_param_from_nvs(rclc_parameter_server_t *server)
{
    nvs_handle_t nvs_handle;
    esp_err_t err = nvs_open("params", NVS_READONLY, &nvs_handle);
    if (err != ESP_OK)
    {
        ESP_LOGW(TAG, "NVS open for read failed (%d). Possibly no stored param1 yet.", err);
        return;
    }

    int64_t param1_val;
    err = nvs_get_i64(nvs_handle, "param1", &param1_val);
    nvs_close(nvs_handle);

    if (err == ESP_OK)
    {
        // Set the server's param1 to the loaded value
        RCCHECK(rclc_parameter_set_int(server, "param1", param1_val));
        ESP_LOGI(TAG, "Loaded param1=%lld from NVS.", (long long)param1_val);
    }
    else if (err == ESP_ERR_NVS_NOT_FOUND)
    {
        ESP_LOGW(TAG, "'param1' not found in NVS. Will use default after this function.");
    }
    else
    {
        ESP_LOGE(TAG, "Failed to load 'param1' from NVS (%d). Will use default after this function.", err);
    }
}

/**
 * @brief Save a single parameter to NVS. We assume NVS is already initialized in app_main().
 */
static esp_err_t save_param_to_nvs(const rcl_interfaces__msg__Parameter * new_param)
{
    nvs_handle_t nvs_handle;
    esp_err_t err = nvs_open("params", NVS_READWRITE, &nvs_handle);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to open NVS namespace 'params' for write (%d)", err);
        return err;
    }

    if (strcmp(new_param->name.data, "param1") == 0)
    {
        // Store 'param1' as int64
        int64_t value = new_param->value.integer_value;
        err = nvs_set_i64(nvs_handle, "param1", value);
        if (err == ESP_OK)
        {
            err = nvs_commit(nvs_handle);
        }
    }
    else
    {
        ESP_LOGW(TAG, "Unknown parameter '%s'. Skipping NVS save.", new_param->name.data);
        // Not an error, just skip storing
        err = ESP_OK;
    }

    nvs_close(nvs_handle);

    if (err == ESP_OK)
    {
        ESP_LOGI(TAG, "Parameter '%s' persisted to NVS.", new_param->name.data);
    }
    else
    {
        ESP_LOGE(TAG, "Error writing param '%s' to NVS (%d)", new_param->name.data, err);
    }
    return err;
}

/**
 * @brief Main entry point. Initializes NVS, transport, and creates micro_ros_task.
 */
void app_main(void)
{
#if defined(RMW_UXRCE_TRANSPORT_CUSTOM)
    // Set up custom serial transport
    // Example UART config
    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };
    uart_param_config(UART_NUM_1, &uart_config);
    uart_driver_install(UART_NUM_1, 1024, 0, 0, NULL, 0);
#endif  // RMW_UXRCE_TRANSPORT_CUSTOM

#if defined(RMW_UXRCE_TRANSPORT_UDP)
    // Initialize networking interface (UDP)
    ESP_ERROR_CHECK(uros_network_interface_initialize());
#endif  // RMW_UXRCE_TRANSPORT_UDP

    // Initialize NVS exactly once here
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to initialize NVS (%d)", err);
    }

    // Create the Micro-ROS task
    xTaskCreate(
        micro_ros_task,
        "uros_task",
        CONFIG_MICRO_ROS_APP_STACK,
        NULL,
        CONFIG_MICRO_ROS_APP_TASK_PRIO,
        NULL
    );
}
