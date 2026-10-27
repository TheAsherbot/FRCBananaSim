/**
 * @file Sha1.cpp
 * @brief Implementation of standalone SHA-1 cryptographic digest and Base64 encoding.
 */

#include "Bridge/Sha1.h"
#include <array>
#include <cstring>

namespace FRCBananaSim::Bridge
{
    namespace
    {
        inline uint32_t LeftRotate(uint32_t value, uint32_t count)
        {
            return (value << count) | (value >> (32 - count));
        }
    }

    std::vector<uint8_t> ComputeSha1(std::string_view input)
    {
        // Initial SHA-1 state constants (FIPS 180-1)
        uint32_t h0 = 0x67452301;
        uint32_t h1 = 0xEFCDAB89;
        uint32_t h2 = 0x98BADCFE;
        uint32_t h3 = 0x10325476;
        uint32_t h4 = 0xC3D2E1F0;

        uint64_t originalBitLength = static_cast<uint64_t>(input.size()) * 8;

        // Build padded message buffer
        std::vector<uint8_t> padded(input.begin(), input.end());
        padded.push_back(0x80); // Append single '1' bit

        // Pad with zeros until message length == 56 mod 64
        while ((padded.size() % 64) != 56)
        {
            padded.push_back(0x00);
        }

        // Append 64-bit big-endian original bit length
        for (int i = 7; i >= 0; --i)
        {
            padded.push_back(static_cast<uint8_t>((originalBitLength >> (i * 8)) & 0xFF));
        }

        // Process message in consecutive 64-byte chunks
        for (size_t chunk = 0; chunk < padded.size(); chunk += 64)
        {
            std::array<uint32_t, 80> w = { 0 };

            // 1. Break 64-byte chunk into sixteen 32-bit big-endian words
            for (size_t t = 0; t < 16; ++t)
            {
                w[t] = (static_cast<uint32_t>(padded[chunk + t * 4]) << 24) |
                    (static_cast<uint32_t>(padded[chunk + t * 4 + 1]) << 16) |
                    (static_cast<uint32_t>(padded[chunk + t * 4 + 2]) << 8) |
                    (static_cast<uint32_t>(padded[chunk + t * 4 + 3]));
            }

            // 2. Extend sixteen words into eighty words
            for (size_t t = 16; t < 80; ++t)
            {
                w[t] = LeftRotate(w[t - 3] ^ w[t - 8] ^ w[t - 14] ^ w[t - 16], 1);
            }

            // 3. Initialize working variables
            uint32_t a = h0;
            uint32_t b = h1;
            uint32_t c = h2;
            uint32_t d = h3;
            uint32_t e = h4;

            // 4. Main 80-round compression loop
            for (size_t t = 0; t < 80; ++t)
            {
                uint32_t f = 0;
                uint32_t k = 0;

                if (t < 20)
                {
                    f = (b & c) | ((~b) & d);
                    k = 0x5A827999;
                }
                else if (t < 40)
                {
                    f = b ^ c ^ d;
                    k = 0x6ED9EBA1;
                }
                else if (t < 60)
                {
                    f = (b & c) | (b & d) | (c & d);
                    k = 0x8F1BBCDC;
                }
                else
                {
                    f = b ^ c ^ d;
                    k = 0xCA62C1D6;
                }

                uint32_t temp = LeftRotate(a, 5) + f + e + k + w[t];
                e = d;
                d = c;
                c = LeftRotate(b, 30);
                b = a;
                a = temp;
            }

            // 5. Add compressed chunk to hash state
            h0 += a;
            h1 += b;
            h2 += c;
            h3 += d;
            h4 += e;
        }

        // Produce final 20-byte digest
        std::vector<uint8_t> digest(20);
        uint32_t parts[5] = { h0, h1, h2, h3, h4 };
        for (int i = 0; i < 5; ++i)
        {
            digest[i * 4] = static_cast<uint8_t>((parts[i] >> 24) & 0xFF);
            digest[i * 4 + 1] = static_cast<uint8_t>((parts[i] >> 16) & 0xFF);
            digest[i * 4 + 2] = static_cast<uint8_t>((parts[i] >> 8) & 0xFF);
            digest[i * 4 + 3] = static_cast<uint8_t>(parts[i] & 0xFF);
        }

        return digest;
    }

    std::string Base64Encode(const std::vector<uint8_t>& data)
    {
        static constexpr char base64Chars[] =
            "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";

        std::string result;
        size_t n = data.size();
        result.reserve(((n + 2) / 3) * 4);

        for (size_t i = 0; i < n; i += 3)
        {
            uint32_t b0 = data[i];
            uint32_t b1 = (i + 1 < n) ? data[i + 1] : 0;
            uint32_t b2 = (i + 2 < n) ? data[i + 2] : 0;

            uint32_t triple = (b0 << 16) | (b1 << 8) | b2;

            result.push_back(base64Chars[(triple >> 18) & 0x3F]);
            result.push_back(base64Chars[(triple >> 12) & 0x3F]);

            if (i + 1 < n)
            {
                result.push_back(base64Chars[(triple >> 6) & 0x3F]);
            }
            else
            {
                result.push_back('=');
            }

            if (i + 2 < n)
            {
                result.push_back(base64Chars[triple & 0x3F]);
            }
            else
            {
                result.push_back('=');
            }
        }

        return result;
    }

    std::string ComputeWebSocketAcceptKey(std::string_view clientKey)
    {
        // Magic RFC 6455 GUID suffix
        static constexpr std::string_view kMagicGuid = "258EAFA5-E914-47DA-95CA-C5AB0DC85B11";

        std::string combined;
        combined.reserve(clientKey.size() + kMagicGuid.size());
        combined.append(clientKey);
        combined.append(kMagicGuid);

        std::vector<uint8_t> sha1Hash = ComputeSha1(combined);
        return Base64Encode(sha1Hash);
    }
}