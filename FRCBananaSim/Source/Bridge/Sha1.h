/**
 * @file Sha1.h
 * @brief Standalone SHA-1 digest and Base64 encoding utilities for RFC 6455 WebSocket handshakes.
 */

#pragma once

#include <cstdint>
#include <string>
#include <string_view>
#include <vector>

namespace FRCBananaSim::Bridge
{
    /**
     * @brief Computes 20-byte SHA-1 digest according to RFC 3174 / FIPS 180-1.
     * @param input Data buffer to hash.
     * @return 20-byte binary hash digest.
     */
    std::vector<uint8_t> ComputeSha1(std::string_view input);

    /**
     * @brief Encodes binary data into standard Base64 string according to RFC 4648.
     * @param data Binary input buffer.
     * @return Base64-encoded string.
     */
    std::string Base64Encode(const std::vector<uint8_t>& data);

    /**
     * @brief Computes standard RFC 6455 Sec-WebSocket-Accept key from client key.
     * @param clientKey Value from incoming Sec-WebSocket-Key HTTP header.
     * @return Calculated accept key for HTTP 101 Switching Protocols response.
     */
    std::string ComputeWebSocketAcceptKey(std::string_view clientKey);
}