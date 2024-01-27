#pragma once

#include "datagrams_defs.hpp"
#include "primatives.hpp"
#include <vector>

DATAGRAMS_NS_HEAD

/**
 * @brief Class for handling control messages.
 * 
 * This class provides functionalities to create, manipulate, and serialize
 * control messages used in communication protocols.
 */
class Control
{
public:
    /**
     * @brief Structure representing the layout of a control message.
     */
    struct Control_t{
        uint8_t     sub_char; ///< Substitution character
        uint8_t     id;       ///<  Message Identifier
        uint8_t     analogs;  ///< Analog values
        uint16_t    pwm_power; ///< PWM power settings
        uint16_t    switched_outputs_power; ///< Power settings for switched outputs
        uint8_t     secondary_supplies;    ///< Secondary supply settings
        uint8_t     open_loop_on; ///< Open loop control settings
        BE_u16      pwm_pair[8];  ///< PWM pair values
        uint8_t     rs232_channel; ///< RS232 channel settings
    }__attribute__((packed));

    /**
     * @brief Constructs a new Control object from a data vector.
     * @param data_vect Vector containing the initial data.
     * @throw std::runtime_error if data_vect size is less than sizeof(Control_t).
     */
    Control(std::vector<uint8_t> data_vect){
        if(data_vect.size() >= sizeof(Control_t)){
            data_vect_ = data_vect;
        }else{
            throw std::runtime_error("Invalid data vector size");
        }
    }

    /**
     * @brief Default constructor. Initializes the control message with default values.
     */
    Control(){
        data_vect_.resize(sizeof(Control_t));
        typePtr()->id = 0x01;
    }

    /**
     * @brief Gets a pointer to the underlying Control_t structure.
     * @return Pointer to Control_t.
     */
    Control_t * typePtr(){
        return reinterpret_cast<Control_t*>(data_vect_.data());
    }

    /**
     * @brief Overloads the arrow operator to provide direct access to Control_t.
     * @return Pointer to Control_t.
     */
    Control_t * operator->(){
        return typePtr();
    }

    /**
     * @brief Sets the open loop control for a specific index.
     * @param index The index to set the open loop control.
     */
    void setOpenLoop(uint8_t index){
        typePtr()->open_loop_on |= (1 << index);
    }

    /**
     * @brief Sets the PWM pair value for a given index.
     * @param index Index of the PWM pair to set.
     * @param scaled_value Scaled value to set for the PWM pair.
     */
    void setPwmPair(uint8_t index, float scaled_value){
        setOpenLoop(index);
        u16 raw_value = scaled_value * 0x3FFF;
        //typePtr()->pwm_pair[index] = raw_value;
        typePtr()->pwm_pair[index].set(raw_value);
    }

    /**
     * @brief Serializes the Control object to a byte vector.
     * @return A std::vector<uint8_t> representing the serialized data.
     */
    std::vector<uint8_t> serialize(){
        return data_vect_;
    }


protected:
    std::vector<uint8_t> data_vect_; ///< Internal storage for control message data.

};

DATAGRAMS_NS_FOOT