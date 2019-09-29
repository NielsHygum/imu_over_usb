//
// Created by nni on 08.08.18.
//

#include "VectornavParser.hpp"
#include <cstring>
#include <android/log.h>
#include <cmath>
#include <Eigen/Geometry>

constexpr char VectornavParser::headerTextStart [];
constexpr size_t VectornavParser::headerGroupLength;
constexpr char VectornavParser::headerTextYMRGroup [];
constexpr char VectornavParser::headerTextYPRGroup [];
constexpr char VectornavParser::headerTextDCMGroup [];
constexpr char VectornavParser::headerEnd;
constexpr char VectornavParser::payLoadEnd;

VectornavParser::VectornavParser()
{
    calculateDCM();
}
/*
void VectornavParser::readDataChunk(const char *data, size_t data_length)
{
    __android_log_print(ANDROID_LOG_DEBUG, "VectornavParser", "read %lu bytes of data", data_length);
    //__android_log_print(ANDROID_LOG_VERBOSE, "VectornavParser", "data: %.*s", static_cast<int>(data_length),data);

    //buffer.write(data, data_length);
    writeNewestDataToBuffer(data, data_length);
    parseNewData();
}
*/
void VectornavParser::doFullParse(const char * data, size_t data_size)
{
    constexpr size_t payload_block_length = 8;
    constexpr size_t number_of_blocks = 12;
    constexpr size_t payload_length = (payload_block_length+1)*number_of_blocks;
    constexpr size_t payload_position = sizeof(headerTextStart)+headerGroupLength+1;

    char payload_block_buffer[payload_block_length+1];

    if(data_size >= payload_position+payload_block_length)
    {
        //parsing yaw
        constexpr size_t payload_block_index_yaw = 0;
        constexpr size_t yaw_position = payload_position+payload_block_index_yaw*(payload_block_length+1);
        memcpy(payload_block_buffer, data+yaw_position, payload_block_length);

        //converts it to prober null terminated cstring
        payload_block_buffer[8] = '\0';
        yaw = atof(payload_block_buffer);
        __android_log_print(ANDROID_LOG_DEBUG, "VectornavParser", "converting block %d with string rep: %s to float %f", static_cast<int>(payload_block_index_yaw), payload_block_buffer, atof(payload_block_buffer));

        //parsing pitch
        constexpr size_t payload_block_index_pitch = 1;
        constexpr size_t pitch_position = payload_position+payload_block_index_pitch*(payload_block_length+1);
        memcpy(payload_block_buffer, data+pitch_position, payload_block_length);

        pitch = atof(payload_block_buffer);
        __android_log_print(ANDROID_LOG_DEBUG, "VectornavParser", "converting block %d with string rep: %s to float %f", static_cast<int>(payload_block_index_pitch), payload_block_buffer, atof(payload_block_buffer));

        //parsing yaw
        constexpr size_t payload_block_index_roll = 2;
        constexpr size_t roll_position = payload_position+payload_block_index_roll*(payload_block_length+1);
        memcpy(payload_block_buffer, data+roll_position, payload_block_length);

        //converts it to prober null terminated cstring
        roll = atof(payload_block_buffer);
        __android_log_print(ANDROID_LOG_DEBUG, "VectornavParser", "converting block %d with string rep: %s to float %f", static_cast<int>(payload_block_index_roll), payload_block_buffer, atof(payload_block_buffer));

        calculateDCM();
    }
}

int VectornavParser::read_new_data(const unsigned char * read_data, int bytes_read)
{
    //readDataChunk(reinterpret_cast<const char *>(read_data), static_cast<size_t >(bytes_read));
    doFullParse(reinterpret_cast<const char *>(read_data), static_cast<size_t >(bytes_read));

    return 0;
}
/*
void VectornavParser::parse(const char *data, size_t data_size)
{

}

void VectornavParser::parseNewData()
{
    switch (mode)
    {
        case ParserMode::SearchingStart :
            //__android_log_print(ANDROID_LOG_DEBUG, "VectornavParser", "searching for start id");
            if (findStart())
            {
                //__android_log_print(ANDROID_LOG_DEBUG, "VectornavParser", "found start");
                mode = ParserMode::ParsingHearderGroup;
            } else {
                break;
            }


        case ParserMode::ParsingHearderGroup :
            //__android_log_print(ANDROID_LOG_DEBUG, "VectornavParser", "parsing header group");
            if (parsedHeaderGroup()) {
                //__android_log_print(ANDROID_LOG_DEBUG, "VectornavParser","parsed header group:");
                mode = ParserMode::ParsingPayload;
            } else {
                break;
            }
        case ParserMode::ParsingPayload : //parse crc before payload
            //__android_log_print(ANDROID_LOG_DEBUG, "VectornavParser", "parsing payload");
            if (parsedPayload())
            {
                //__android_log_print(ANDROID_LOG_DEBUG, "VectornavParser", "parsed payload");
                //mode = ParserMode::ParsingCRC;
                mode = ParserMode::SearchingStart;
            }

            break;
    }
}


void VectornavParser::resetParser()
{
    mode = ParserMode::SearchingStart;
}

void VectornavParser::failedParsing()
{
    mode = ParserMode::SearchingStart;
}

const char *VectornavParser::findValueInBuffer(size_t search_position, int value)
{
    char *chp = nullptr;

    if (head < search_position)
    {
        // search up to tail position
        chp = (char *) memchr(data  + search_position, value,
                              sizeOfBuffer() - search_position);

        if (chp == nullptr) {
            chp = (char *) memchr(data, value, head);
        }
    }
    else if ( head > search_position)
    {
        chp = (char *) memchr(data  + search_position, value,
                              sizeOfBuffer() - head);
    }

    return chp;
}

std::pair<HeaderStartState, size_t> VectornavParser::findHeaderStartInBuffer()
{

    size_t search_position = tail;
    //for (size_t search_position = tail; search_position != head; search_position= search_position+1 % sizeOfBuffer())
    while(search_position != head)
    {
        const char *result = findValueInBuffer(search_position, headerTextStart[0]);


        if (result != nullptr)
        {
            //check if next block is ok
            size_t hit_position = result - data;

            if(hit_position == head)
                return std::make_pair(HeaderStartState::Partial, hit_position);

            bool valid_start = true;

            for (size_t n = 1; n < sizeof(headerTextStart)-1; n++)
            {
                size_t buffer_test_position = hit_position + n % sizeOfBuffer();

                if (buffer_test_position == head)
                {
                    //reached end of buffer
                    return std::make_pair(HeaderStartState::Partial, hit_position);
                } else if (headerTextStart[n] != data[buffer_test_position]) {
                    valid_start = false;
                    search_position = hit_position + 1 % sizeOfBuffer();
                    break;
                }

            }

            if(valid_start == true)
            {
                return std::make_pair(HeaderStartState::Full, hit_position);
            }

        } else {
            return std::make_pair(HeaderStartState::None, head);
        }
    }

    // never reached but compiler likes it
    return std::make_pair(HeaderStartState::None, head);
}

bool VectornavParser::findStart()
{
    std::pair<HeaderStartState, size_t> result = findHeaderStartInBuffer();

    tail = result.second;

    switch (result.first)
    {
        case HeaderStartState::None : //did not find anything at all in data chunk
            //__android_log_print(ANDROID_LOG_DEBUG, "VectornavParser", "found no start id");
            return false;
        case HeaderStartState::Partial : //found partial hit at end of data chunk
            //__android_log_print(ANDROID_LOG_DEBUG, "VectornavParser", "found partial start id at position %d", tail);
            return false;
        case HeaderStartState::Full :
            //__android_log_print(ANDROID_LOG_DEBUG, "VectornavParser", "found full start id at position %d", tail);
            return true;
    }

}

bool VectornavParser::parsedHeaderGroup()
{
    size_t headerGroupStart = tail+sizeof(headerTextStart) % sizeOfBuffer(); //minus 1 because we dont want the

    //__android_log_print(ANDROID_LOG_DEBUG, "VectornavParser", "data: %.*s", static_cast<int>(headerGroupLength+sizeof(headerTextStart)),data+tail);

    //check that entire header group can be read
    if(headerGroupLength+sizeof(headerTextStart) > currentFilledCapacity())
        return false; // return to read more data


    //find header group
    if(dataEqualBuffer( headerTextYPRGroup, headerGroupStart,headerGroupLength))
    {
        payload_type == HeaderGroup::YPR;
        return true;
    }


    if(dataEqualBuffer( headerTextYMRGroup, headerGroupStart,headerGroupLength))
    {
        payload_type == HeaderGroup::YMR;
        return true;
    }

    if(dataEqualBuffer( headerTextDCMGroup, headerGroupStart,headerGroupLength))
    {
        payload_type == HeaderGroup::DCM;
        return true;
    }


    failedParsing();

    return false;
}

bool VectornavParser::parsedPayload()
{
    switch(payload_type)
    {
        case HeaderGroup::YMR :
            return parsedYMRPayload();
        case HeaderGroup::YPR :
            __android_log_print(ANDROID_LOG_ERROR, "VectornavParser", "parsing payload of type YPR not implemented");
            break;
        case HeaderGroup::DCM :
            __android_log_print(ANDROID_LOG_ERROR, "VectornavParser", "parsing payload of type DCM not implemented");
            break;
    }

    failedParsing();

    return false;
}


bool VectornavParser::parsedYMRPayload()
{
    constexpr size_t payload_block_length = 8;
    constexpr size_t number_of_blocks = 12;
    constexpr size_t payload_length = payload_block_length*number_of_blocks;

    size_t payload_start_position = tail+sizeof(headerTextStart)+headerGroupLength+1 % sizeOfBuffer();

    // check if all payload is available

    if(payload_block_length+headerGroupLength+sizeof(headerTextStart)+1 > currentFilledCapacity())
        return false;

    yaw = parsePayloadFloatItem(0,payload_start_position);
    pitch = parsePayloadFloatItem(1,payload_start_position);
    roll = parsePayloadFloatItem(2,payload_start_position);

    float yaw_copy = yaw;

    __android_log_print(ANDROID_LOG_DEBUG, "VectornavParser", "parsed yaw: %f", yaw_copy);
    //__android_log_print(ANDROID_LOG_ERROR, "VectornavParser", "parsed pitch: %f", yaw);
    //__android_log_print(ANDROID_LOG_ERROR, "VectornavParser", "parsed roll: %f", yaw);

    failedParsing();

    return false;
}

float VectornavParser::parsePayloadFloatItem(size_t payload_block_index, size_t payload_start_position)
{
    constexpr size_t payload_block_length = 8;

    char payload_block_buffer[payload_block_length+1];

    size_t payload_position = payload_start_position + payload_block_index*(payload_block_length+1) % sizeOfBuffer();

    if (payload_block_index > head)
    {
        memcpy(payload_block_buffer, data+payload_position, sizeOfBuffer()-payload_position);
        memcpy(payload_block_buffer+sizeOfBuffer()-payload_position, data, payload_block_length-(sizeOfBuffer()-payload_position));
    } else {
        memcpy(payload_block_buffer, data+payload_position, payload_block_length);
    }

    //converts it to prober null terminated cstring
    payload_block_buffer[8] = '\0';

    //__android_log_print(ANDROID_LOG_ERROR, "VectornavParser", "converting block %d with string rep: %s to float %f", static_cast<int>(payload_block_index), payload_block_buffer, atof(payload_block_buffer));

    return atof(payload_block_buffer);
}

bool VectornavParser::dataEqualBuffer(const char * data_to_check, size_t buffer_position, size_t data_size)
{
    if(buffer_position > head) {
        if(memcmp(data+buffer_position, data_to_check, sizeOfBuffer()-buffer_position) == 0)
        {
            if(memcmp(data, data_to_check+sizeOfBuffer()-buffer_position, head) == 0)
                return true;
        } else {
            return false;
        }
    }

    if(buffer_position < head) {
        return (memcmp(data+buffer_position, data_to_check, data_size) == 0);
    }

    return false;
}*/

float VectornavParser::getYaw() const
{
    return yaw;
}

float VectornavParser::getPitch() const
{
    return pitch;
}

float VectornavParser::getRoll() const
{
    return roll;
}

Eigen::Matrix3f VectornavParser::getDCM() const
{
    return _DCM;
}

void VectornavParser::calculateDCM()
{
    float yaw_rad = yaw/180.0f*3.141592;
    float pitch_rad = pitch/180.0f*3.141592;
    float roll_rad = roll/180.0f*3.141592;

    _DCM = Eigen::AngleAxisf(-yaw_rad, Eigen::Vector3f::UnitY()).toRotationMatrix()
    * Eigen::AngleAxisf(pitch_rad,  Eigen::Vector3f::UnitX()).toRotationMatrix()
    * Eigen::AngleAxisf(-roll_rad, Eigen::Vector3f::UnitZ()).toRotationMatrix();
}

// Calculates the 8-bit checksum for the given byte sequence.
unsigned char calculateChecksum(const unsigned char data[], size_t length)
{
    unsigned char cksum = 0;

    for(size_t i=0; i<length; i++)
        cksum ^= data[i];

    return cksum;
}

// Calculates the 16-bit CRC for the given ASCII or binary message.
unsigned short calculateCRC(unsigned char data[], size_t length)
{
    unsigned short crc = 0;

    for(size_t i=0;i<length; i++)
    {
        crc = (unsigned char)(crc >> 8) | (crc << 8);
        crc ^= data[i];
        crc ^= (unsigned char)(crc & 0xff) >> 4;
        crc ^= crc << 12;
        crc ^= (crc & 0x00ff) << 5;
    }

    return crc;

}

