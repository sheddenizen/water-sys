#ifndef MQTT_CLIENT_H
#define MQTT_CLIENT_H

typedef unsigned short mqtt_size_t;

// Callback function for subscribed messages
typedef void (*mqtt_msg_rx_fn)(char const * topic, mqtt_size_t topic_len,
                                char const * msg, mqtt_size_t msg_len);

typedef struct mqtt_subscription
{
    char const * topic;
    mqtt_msg_rx_fn handler;

} mqtt_subscription_t;

// Serialization function to go with posted data, invoked on dequeuing of data to be posted
typedef mqtt_size_t (*mqtt_msg_serialize_fn)(char const * in_data, mqtt_size_t in_data_size, char * out_buf, mqtt_size_t out_buf_size);

// Initialize mqtt task
// Subscription list must persist beyond fn call
void mqtt_init(mqtt_subscription_t const subscriptions[], size_t num_subscriptions, mqtt_size_t data_size);

// Notify change in network conectivity status
void mqtt_connection_notify(bool is_connected);

// Post data to topic, queue if necessary
// topic - topic name - not copied, should be static
// in_data - pointer to raw data - will be copied to queue
// can_block - if true, block if queue full, otherwise discard
// returns true if successfully queued
bool mqtt_post_data(char const * topic, mqtt_msg_serialize_fn serializer, char const * in_data, mqtt_size_t in_data_size, uint8_t qos, bool retain, bool can_block);


#endif // MQTT_CLIENT_H
