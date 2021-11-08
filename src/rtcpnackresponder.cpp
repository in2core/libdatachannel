/**
 * Copyright (c) 2020 Filip Klembara (in2core)
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA
 */

#if RTC_ENABLE_MEDIA

#include "rtcpnackresponder.hpp"

#include "impl/internals.hpp"
#include "impl/message.hpp"

#include <cassert>
#include <chrono>
#include <vector>

using namespace std::chrono;

namespace rtc {

RtcpNackResponder::OutgoingStorage::Element::Element(binary_ptr packet, uint16_t sequenceNumber,
                                             shared_ptr<Element> next)
: packet(packet), sequenceNumber(sequenceNumber), next(next) {}

unsigned RtcpNackResponder::OutgoingStorage::size() { return storage.size(); }

RtcpNackResponder::OutgoingStorage::OutgoingStorage(unsigned _maximumSize) : maximumSize(_maximumSize) {
	assert(maximumSize > 0);
	storage.reserve(maximumSize);
}

optional<binary_ptr> RtcpNackResponder::OutgoingStorage::get(uint16_t sequenceNumber) {
	auto position = storage.find(sequenceNumber);
	return position != storage.end() ? std::make_optional(storage.at(sequenceNumber)->packet)
	                                 : nullopt;
}

void RtcpNackResponder::OutgoingStorage::store(binary_ptr packet) {
	if (!packet || packet->size() < 12) {
		return;
	}
	auto rtp = reinterpret_cast<RTP *>(packet->data());
	auto sequenceNumber = rtp->seqNumber();

	assert((storage.empty() && !oldest && !newest) || (!storage.empty() && oldest && newest));

	if (size() == 0) {
		newest = std::make_shared<Element>(packet, sequenceNumber);
		oldest = newest;
	} else {
		auto current = std::make_shared<Element>(packet, sequenceNumber);
		newest->next = current;
		newest = current;
	}

	storage.emplace(sequenceNumber, newest);

	if (size() > maximumSize) {
		assert(oldest);
		if (oldest) {
			storage.erase(oldest->sequenceNumber);
			oldest = oldest->next;
		}
	}
}


RtcpNackResponder::IncomingStorage::Element::Element(binary_ptr packet, long long time_ms)
: packet(packet), time_ms(time_ms) {}

RtcpNackResponder::IncomingStorage::IncomingStorage(long long maximumWaitTime_ms)
:maximumWaitTime_ms(maximumWaitTime_ms) {
    currentStorage = &storage1;
    overflowStorage = &storage2;
}

optional<std::pair<uint16_t, bool>> RtcpNackResponder::IncomingStorage::getPreviousSequenceNumberWithOverflow(uint16_t sequence_number) {
    uint16_t previous_sequence_number;
    bool overflow = false;
    if (previousReportedSequenceNumber.has_value()) {
        previous_sequence_number = *previousReportedSequenceNumber;
        if ((previous_sequence_number >> 15 == 1) && (sequence_number >> 15 == 0)) {
            // overflow
            overflow = true;
        } else if (previous_sequence_number > sequence_number) {
            return nullopt;
        }
    } else {
        previous_sequence_number = sequence_number - 1;
        previousReportedSequenceNumber = previous_sequence_number;
        overflow = sequence_number == 0;
    }
    return {{previous_sequence_number, overflow}};
}

std::map<uint16_t, shared_ptr<RtcpNackResponder::IncomingStorage::Element>>::iterator RtcpNackResponder::IncomingStorage::addPacket(binary_ptr packet, uint16_t sequence_number, long long time_ms, std::map<uint16_t, shared_ptr<Element>> * used_storage) {
    const auto element = std::make_shared<Element>(packet, time_ms);
    const auto element_iterator = used_storage->emplace(sequence_number, element).first;
    auto next_element_iterator = std::next(element_iterator);
    if (used_storage == currentStorage) {
        if (next_element_iterator == currentStorage->end()) {
            next_element_iterator = overflowStorage->begin();
            if (next_element_iterator != overflowStorage->end()) {
                element->time_ms = next_element_iterator->second->time_ms;
            }
        } else {
            element->time_ms = next_element_iterator->second->time_ms;
        }
    } else if (next_element_iterator != overflowStorage->end()) {
        element->time_ms = next_element_iterator->second->time_ms;
    }
    return element_iterator;
};

uint16_t RtcpNackResponder::IncomingStorage::getPreviousReceivedSequenceNumber(std::map<uint16_t, shared_ptr<RtcpNackResponder::IncomingStorage::Element>>::iterator element_iterator, std::map<uint16_t, shared_ptr<RtcpNackResponder::IncomingStorage::Element>> * used_storage) {
    optional<uint16_t> prev_seq_no = nullopt;
    if (used_storage == overflowStorage) {
        assert(!overflowStorage->empty());
        if (element_iterator == overflowStorage->begin()) {
            if (!currentStorage->empty()) {
                if (currentStorage->size() > 1) {
                    prev_seq_no = std::prev(currentStorage->end())->first;
                } else {
                    prev_seq_no = currentStorage->begin()->first;
                }
            }
        } else {
            prev_seq_no = std::prev(element_iterator)->first;
        }
    } else {
        assert(used_storage == currentStorage);
        assert(!currentStorage->empty());
        if (element_iterator != currentStorage->begin()) {
            prev_seq_no = std::prev(element_iterator)->first;
        }
    }
    if (prev_seq_no.has_value()) {
        return *prev_seq_no;
    } else if (previousReportedSequenceNumber.has_value()) {
        return *previousReportedSequenceNumber;
    } else {
        return element_iterator->first - 1;
    }
}

optional<std::pair<uint16_t, uint16_t>> RtcpNackResponder::IncomingStorage::generateNack(uint16_t previous_recieved_sequence_number, uint16_t sequence_number) {
    const uint16_t prev_seq_no = sequence_number - 1;
    uint16_t counter = 0;
    const uint16_t max_nack_packet_count = 17;
    uint16_t offset = 0;
    while (offset < max_nack_packet_count && uint16_t(prev_seq_no - offset) != previous_recieved_sequence_number && requestedSequenceNumbers.find(prev_seq_no - offset) != requestedSequenceNumbers.end()) {
        offset += 1;
    }
    counter = offset;
    while (counter < max_nack_packet_count && uint16_t(prev_seq_no - counter) != previous_recieved_sequence_number && requestedSequenceNumbers.find(prev_seq_no - counter) == requestedSequenceNumbers.end()) {
        requestedSequenceNumbers.emplace(prev_seq_no - counter);
        counter += 1;
    }
    if (counter > offset) {
        const uint16_t pid = prev_seq_no - counter + 1;
        const uint16_t blp = 0xFFFF >> (max_nack_packet_count - counter + offset);
        return std::make_pair(pid, blp);
    }
    return nullopt;
}

ChainedMessagesProduct RtcpNackResponder::IncomingStorage::getIncomingPackets(long long time_ms) {
    const auto received_packets = make_chained_messages_product();
    if (!previousReportedSequenceNumber.has_value()) {
        return received_packets;
    }
    const auto last_reported_sequence_number = *previousReportedSequenceNumber;

    const auto first_element_iterator = currentStorage->begin();
    uint16_t prevSeqNumber = last_reported_sequence_number;
    auto curr_iterator = first_element_iterator;
    while (curr_iterator != currentStorage->end() && (uint16_t(prevSeqNumber + 1) == curr_iterator->first || curr_iterator->second->time_ms + maximumWaitTime_ms < time_ms)) {
        received_packets->push_back(curr_iterator->second->packet);
        prevSeqNumber = curr_iterator->first;
        curr_iterator = currentStorage->erase(curr_iterator);
    }

    bool swap_storage = false;
    if (curr_iterator == currentStorage->end()) {
        curr_iterator = overflowStorage->begin();
        swap_storage = curr_iterator != overflowStorage->end() && (uint16_t(prevSeqNumber + 1) == curr_iterator->first || curr_iterator->second->time_ms + maximumWaitTime_ms < time_ms);
        while (curr_iterator != overflowStorage->end() && (uint16_t(prevSeqNumber + 1) == curr_iterator->first || curr_iterator->second->time_ms + maximumWaitTime_ms < time_ms)) {
            received_packets->push_back(curr_iterator->second->packet);
            prevSeqNumber = curr_iterator->first;
            curr_iterator = overflowStorage->erase(curr_iterator);
        }
    }

    if (!received_packets->empty()) {
        previousReportedSequenceNumber = prevSeqNumber;
        auto first_nack_request = requestedSequenceNumbers.begin();
        while (first_nack_request != requestedSequenceNumbers.end() && *first_nack_request <= prevSeqNumber) {
            first_nack_request = std::next(first_nack_request);
        }
        requestedSequenceNumbers.erase(requestedSequenceNumbers.begin(), first_nack_request);
    }

    if (swap_storage) {
        auto tmp = overflowStorage;
        overflowStorage = currentStorage;
        currentStorage = tmp;
    }
    return received_packets;
}

std::pair<ChainedMessagesProduct, std::optional<std::pair<uint16_t, uint16_t>>> RtcpNackResponder::IncomingStorage::store(binary_ptr packet, long long time_ms) {
    if (!packet || packet->size() < 12) {
        return {make_chained_messages_product(), nullopt};
    }

    const auto rtp = reinterpret_cast<RTP *>(packet->data());
    const uint16_t sequence_number = rtp->seqNumber();

    auto previous_sequence_and_overflow = getPreviousSequenceNumberWithOverflow(sequence_number);

    if (!previous_sequence_and_overflow.has_value()) {
        return {make_chained_messages_product(), nullopt};
    }

    uint16_t last_reported_sequence_number = previous_sequence_and_overflow->first;
    bool overflow = previous_sequence_and_overflow->second;

    const auto used_storage = overflow ? overflowStorage : currentStorage;

    const auto element_iterator = addPacket(packet, sequence_number, time_ms, used_storage);

    assert(element_iterator != used_storage->end());
    uint16_t previous_recieved_sequence_number = getPreviousReceivedSequenceNumber(element_iterator, used_storage);

    const auto nack = generateNack(previous_recieved_sequence_number, sequence_number);

    const auto storage = currentStorage->empty() ? overflowStorage : currentStorage;
    if (!storage->empty()) {
        auto current = storage->begin();
        if (current->second->time_ms + maximumWaitTime_ms < time_ms) {
            last_reported_sequence_number = current->first - 1;
            previousReportedSequenceNumber = last_reported_sequence_number;
        }
    }

    const auto received_packets = getIncomingPackets(time_ms);
    return {received_packets, nack};
}

RtcpNackResponder::RtcpNackResponder(unsigned maxStoredPacketCount, long long maxWaitTime)
    : MediaHandlerElement(), outgoingStorage(std::make_shared<OutgoingStorage>(maxStoredPacketCount)),
	incomingStorage(std::make_shared<IncomingStorage>(maxWaitTime)){}

ChainedIncomingControlProduct
RtcpNackResponder::processIncomingControlMessage(message_ptr message) {
	auto packets = make_chained_messages_product();

	unsigned int i = 0;
	while (i < message->size()) {
		auto nack = reinterpret_cast<RTCP_NACK *>(message->data() + i);
		i += nack->header.header.lengthInBytes();
		// check if rtcp is nack
		if (nack->header.header.payloadType() != 205 || nack->header.header.reportCount() != 1) {
			continue;
		}

		auto fieldsCount = nack->getSeqNoCount();

		std::vector<uint16_t> missingSequenceNumbers{};
		for (unsigned int i = 0; i < fieldsCount; i++) {
			auto field = nack->parts[i];
			auto newMissingSeqenceNumbers = field.getSequenceNumbers();
			missingSequenceNumbers.insert(missingSequenceNumbers.end(),
			                              newMissingSeqenceNumbers.begin(),
			                              newMissingSeqenceNumbers.end());
		}
		packets->reserve(packets->size() + missingSequenceNumbers.size());
		for (auto sequenceNumber : missingSequenceNumbers) {
			auto optPacket = outgoingStorage->get(sequenceNumber);
			if (optPacket.has_value()) {
				auto packet = optPacket.value();
				packets->push_back(packet);
			}
		}
	}

	if (!packets->empty()) {
		return {message, ChainedOutgoingProduct(packets)};
	} else {
		return {message, nullopt};
	}
}

ChainedOutgoingProduct
RtcpNackResponder::processOutgoingBinaryMessage(ChainedMessagesProduct messages,
                                                message_ptr control) {
	for (auto message : *messages) {
		outgoingStorage->store(message);
	}
	return {messages, control};
}


ChainedIncomingProduct
RtcpNackResponder::processIncomingBinaryMessage(ChainedMessagesProduct messages) {
	const auto current_time = duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count();
    const ChainedMessagesProduct incoming = make_chained_messages_product();
    ChainedOutgoingProduct outgoing = ChainedOutgoingProduct();
    std::vector<std::pair<uint32_t, std::pair<uint16_t, uint16_t>>> nacks_data{};

	for (auto message : *messages) {
        if (message->size() < 12) {
            continue;
        }
        const auto messageAndNack = incomingStorage->store(message, current_time);
        const auto incomingMessage = messageAndNack.first;

        std::copy(incomingMessage->begin(), incomingMessage->end(), std::back_inserter(*incoming));

        const auto optNack = messageAndNack.second;
        if (optNack) {
            RTP * rtp = reinterpret_cast<RTP *>(message->data());
            const auto nack = *optNack;
            nacks_data.push_back({rtp->ssrc(), nack});
        }
	}
    const ChainedMessagesProduct incoming_packets = incoming->empty() ? nullptr : incoming;
    if (!nacks_data.empty()) {
        const auto nack_size = RTCP_NACK::Size(1);
        const auto msg = make_message(nacks_data.size() * nack_size, Message::Type::Control);
        for (unsigned i = 0; i < nacks_data.size(); i += 1) {
            const auto nack = reinterpret_cast<RTCP_NACK *>(msg->data() + nack_size * i);
            const auto nack_data = nacks_data[i];
            nack->parts[0].setPid(nack_data.second.first);
            nack->parts[0].setBlp(nack_data.second.second);
            nack->preparePacket(nack_data.first, 1);
        }
        return {incoming_packets, {nullptr, msg}};
    } else {
        return {incoming_packets};
    }
}

} // namespace rtc

#endif /* RTC_ENABLE_MEDIA */
