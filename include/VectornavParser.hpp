//
// Created by nni on 11.10.18.
//

#ifndef BINAURALAUDIO_VECTORNAVPARSER_HPP
#define BINAURALAUDIO_VECTORNAVPARSER_HPP

#include <cstddef>
#include <utility>
#include <basic_ring_buffer.hpp>
#include <ftdi_imu.h>
#include <Eigen/Dense>

enum class ParserMode  {SearchingStart, ParsingHearderGroup, ParsingPayload, ParsingCRC};
enum class HeaderStartState {None, Partial, Full};
enum class HeaderGroup {YMR, YPR, DCM};

class VectornavParser : public RingBuffer<1024>, public FTDI_IMU
{
private:

    ParserMode mode = ParserMode::SearchingStart;
    HeaderGroup payload_type;

    static constexpr char headerTextStart [3] = {'$', 'V', 'N'};//{"$VN"};
    static constexpr size_t headerGroupLength = 3;
    static constexpr char headerTextYMRGroup [3] = {'Y','M','R'};//{"YMR"};
    static constexpr char headerTextYPRGroup [3] = {'Y', 'P', 'R'};//{"YPR"};
    static constexpr char headerTextDCMGroup [3] = {'D', 'C','M'};//{"DCM"};

    static constexpr char headerEnd = ',';

    static constexpr char payLoadEnd = '*';

/*
    void parse(const char * data, size_t data_size);
    void parse(const char byte);
    void parseNewData();

    bool findStart();
    bool parsedHeaderGroup();
    bool parsedPayload();
    bool parsedYMRPayload();
    float parsePayloadFloatItem(size_t payload_block_index, size_t payload_start_position);
    bool parsedCRC();

    void resetParser();
    void failedParsing();

    bool dataEqualBuffer(const char * data_to_check, size_t buffer_position, size_t data_size);

    const char * findValueInBuffer(size_t search_position, int value);
    std::pair<HeaderStartState, size_t> findHeaderStartInBuffer();
*/
    std::atomic<float> yaw = {0.0f};
    std::atomic<float> pitch = {0.0f};
    std::atomic<float> roll = {0.0f};
    Eigen::Matrix3f _DCM;

    int read_new_data(const unsigned char * read_data, int bytes_read) override;

    void calculateDCM();

public:

    VectornavParser();

  //  void readDataChunk(const char * data, size_t data_size);

    void doFullParse(const char * data, size_t data_size);

    float getYaw() const;
    float getPitch() const;
    float getRoll() const;
    Eigen::Matrix3f getDCM() const;
};

unsigned char calculateChecksum(const unsigned char data[], size_t length);
unsigned short calculateCRC(unsigned char data[], size_t length);

#endif //BINAURALAUDIO_VECTORNAVPARSER_HPP
