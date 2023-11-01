#include "base_controller/serial_port_communication.h"

void serialInit()
{
    
    serial::Timeout to = serial::Timeout::simpleTimeout(100);
    serial::parity_t pt = serial::parity_t::parity_none;
    serial::bytesize_t bt = serial::bytesize_t::eightbits;
    serial::flowcontrol_t ft = serial::flowcontrol_t::flowcontrol_none;
    serial::stopbits_t st = serial::stopbits_t::stopbits_one;
    
    sp.setPort("/dev/ttyUSB0");
    sp.setBaudrate(115200);
    sp.setParity(pt);
    sp.setBytesize(bt);
    sp.setFlowcontrol(ft);
    sp.setStopbits(st);
    sp.setTimeout(to);
 
    try
    {
        // Open the serial port
        sp.open();
    }
    catch (serial::IOException& e)
    {
        ROS_ERROR_STREAM("Unable to open port.");
        return;
    }
}

uint8_t float_to_hex(float dec_data)
{
    uint8_t hex_rotate_speed;
    string hex_string;
    stringstream ss;

    ss << std::hex << int(dec_data) << endl;
    ss >> hex_string;
    hex_rotate_speed = strtol(hex_string.c_str(),NULL,16);
    
    return hex_rotate_speed;
}

int hex_to_int(uint8_t hex_rotate_speed1,uint8_t hex_rotate_speed2)
{
    int rotate_speed = (int)(hex_rotate_speed1) * 256 + (int)(hex_rotate_speed2);

    // For the backward rotate speed
    if (rotate_speed >= 55535 && rotate_speed <= 65535)
    {
        rotate_speed = 65535 - rotate_speed;
        float percentage = rotate_speed / 10000.0;
        rotate_speed = percentage * -3000.0;
        return rotate_speed;
    }
    else if (rotate_speed >= 0 && rotate_speed <= 10000)  // For the forward rotate speed
    {
        float percentage = rotate_speed / 10000.0;
        rotate_speed = percentage * 3000.0;
        return rotate_speed;
    }
    else
    {
        ROS_ERROR("The received rotate speeds are not correct!");
        return 0;
    }
}


float transformed_positive_rotate_speed(float rotate_speed)
{
    float percentage = rotate_speed / 3000.0;
    float dec_result = percentage * 10000.0;
    return dec_result;
}

float transformed_negative_rotate_speed(float rotate_speed)
{
    rotate_speed = abs(rotate_speed);
    float percentage = rotate_speed / 3000.0;
    float dec_result = percentage * 10000.0;
    dec_result = 65535 - dec_result;  // 65535 -> 0xFF 0xFF
    return dec_result;
}

void sendRotateSpeed(float left_rotate_speed, float right_rotate_speed)
{
    uint8_t data[12];

    // Drive the two motors together
    data[0] = 0xE0;
    data[1] = 0x03;
    data[2] = 0x00;
    data[3] = 0x00;

    // Calculate the left rotate speed
    if (left_rotate_speed >= 0)
    {
        data[4] = 0x00;
        data[5] = 0x00;
        left_rotate_speed = transformed_positive_rotate_speed(left_rotate_speed);

        if (left_rotate_speed <= 255)
        {
            data[6] = 0x00;
            data[7] = float_to_hex(left_rotate_speed);
        }
        else
        {
            int carry_bit = 0;
            float temp = left_rotate_speed;
            while (temp > 255)
            {
                temp -= 256;
                carry_bit++;
            }
            data[6] = float_to_hex(carry_bit);
            data[7] = float_to_hex(temp);
        }
    }
    else
    {
        data[4] = 0xFF;
        data[5] = 0xFF;
        left_rotate_speed = transformed_negative_rotate_speed(left_rotate_speed);

        if (left_rotate_speed <= 255)
        {
            data[6] = 0x00;
            data[7] = float_to_hex(left_rotate_speed);
        }
        else
        {
            int carry_bit = 0;
            float temp = left_rotate_speed;
            while (temp > 255)
            {
                temp -= 256;
                carry_bit++;
            }
            data[6] = float_to_hex(carry_bit);
            data[7] = float_to_hex(temp);
        }
    }
    
    // Calculate the right rotate speed
    if (right_rotate_speed >= 0)
    {
        data[8] = 0x00;
        data[9] = 0x00;
        right_rotate_speed = transformed_positive_rotate_speed(right_rotate_speed);

        if (right_rotate_speed <= 255)
        {
            data[10] = 0x00;
            data[11] = float_to_hex(right_rotate_speed);
        }
        else
        {
            int carry_bit = 0;
            float temp = right_rotate_speed;
            while (temp > 255)
            {
                temp -= 256;
                carry_bit++;
            }
            data[10] = float_to_hex(carry_bit);
            data[11] = float_to_hex(temp);
        }
    }
    else
    {
        data[8] = 0xFF;
        data[9] = 0xFF;
        right_rotate_speed = transformed_negative_rotate_speed(right_rotate_speed);

        if (right_rotate_speed <= 255)
        {
            data[10] = 0x00;
            data[11] = float_to_hex(right_rotate_speed);
        }
        else
        {
            int carry_bit = 0;
            float temp = right_rotate_speed;
            while (temp > 255)
            {
                temp -= 256;
                carry_bit++;
            }
            data[10] = float_to_hex(carry_bit);
            data[11] = float_to_hex(temp);
        }
    }

    // Print the data
    ROS_INFO("%x %x %x %x %x %x %x %x %x %x %x %x"
             ,data[0],data[1],data[2],data[3],data[4],data[5],
              data[6],data[7],data[8],data[9],data[10],data[11]);

    // send data through serial port
    sp.write(data,12);
}

bool receiveRotateSpeed()
{
    // TODO determine the param!
    sleep(0.1);

    int length = 0;
    uint8_t data[70] = {0x00};
    uint8_t* ptr = data;
    
    // Read the data through serial port
    length = sp.read(data,60);
    ROS_INFO("The length of received data is %d",length);
    if (length < 12)
    {
        ROS_ERROR("Do not receieve enough data, the interval is too short.");
        return false;
    }
    // Point for the twelfth to last
    ptr = ptr + (length - 1 - 11);
    for (int i = 0; i < length - 12; i++)
    {
        if (*ptr == 0xEE)
        {
            LEFT_ROTATE_SPEED_NOW = hex_to_int(*(ptr+1), *(ptr+2));
            RIGHT_ROTATE_SPEED_NOW = hex_to_int(*(ptr+3), *(ptr+4));

            ROS_INFO("The left wheel rotate speed is %d now", LEFT_ROTATE_SPEED_NOW);
            ROS_INFO("The right wheel rotate speed is %d now", RIGHT_ROTATE_SPEED_NOW);
            return true;
        }
        ptr--;
    }
    ROS_ERROR("The received data is not correct heartbeat data");
    return false;
}