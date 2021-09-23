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

#ifndef RTC_RTCP_NACK_RESPONDER_H
#define RTC_RTCP_NACK_RESPONDER_H

#if RTC_ENABLE_MEDIA

#include "mediahandlerelement.hpp"

#include <queue>
#include <unordered_map>
#include <map>
#include <set>

namespace rtc {

class RTC_CPP_EXPORT RtcpNackResponder final : public MediaHandlerElement {

	/// Packet storage
	class RTC_CPP_EXPORT OutgoingStorage {

		/// Packet storage element
		struct RTC_CPP_EXPORT Element {
			Element(binary_ptr packet, uint16_t sequenceNumber, shared_ptr<Element> next = nullptr);
			const binary_ptr packet;
			const uint16_t sequenceNumber;
			/// Pointer to newer element
			shared_ptr<Element> next = nullptr;
		};

		/// Oldest packet in storage
		shared_ptr<Element> oldest = nullptr;
		/// Newest packet in storage
		shared_ptr<Element> newest = nullptr;

		/// Inner storage
		std::unordered_map<uint16_t, shared_ptr<Element>> storage{};

		/// Maximum storage size
		const unsigned maximumSize;

		/// Returnst current size
		unsigned size();

	public:
		static const unsigned defaultMaximumSize = 512;

		OutgoingStorage(unsigned _maximumSize);

		/// Returns packet with given sequence number
		optional<binary_ptr> get(uint16_t sequenceNumber);

		/// Stores packet
		/// @param packet Packet
		void store(binary_ptr packet);
	};
	/// Incoming Packet storage
	class RTC_CPP_EXPORT IncomingStorage {

		/// Packet storage element
		struct RTC_CPP_EXPORT Element {
			Element(binary_ptr packet, long long time_ms);
			const binary_ptr packet;
			long long time_ms;
		};

		std::map<uint16_t, shared_ptr<Element>> storage1{};
		std::map<uint16_t, shared_ptr<Element>> storage2{};
		std::map<uint16_t, shared_ptr<Element>> * currentStorage;
		std::map<uint16_t, shared_ptr<Element>> * overflowStorage;
		optional<uint16_t> previousReportedSequenceNumber = nullopt;
		std::set<uint16_t> requestedSequenceNumbers{};

		optional<std::pair<uint16_t, bool>> getPreviousSequenceNumberWithOverflow(uint16_t sequence_number);
		std::map<uint16_t, shared_ptr<Element>>::iterator addPacket(binary_ptr packet, uint16_t sequence_number, long long time_ms, std::map<uint16_t, shared_ptr<Element>> * used_storage);
		uint16_t getPreviousReceivedSequenceNumber(std::map<uint16_t, shared_ptr<Element>>::iterator element_iterator, std::map<uint16_t, shared_ptr<Element>> * used_storage);
		optional<std::pair<uint16_t, uint16_t>> generateNack(uint16_t previous_recieved_sequence_number, uint16_t sequence_number);
		ChainedMessagesProduct getIncomingPackets(long long time_ms);

		long long maximumWaitTime_ms;
	public:
		IncomingStorage(long long maximumWaitTime_ms);
		/// Stores packet
		/// @param packet Packet
		std::pair<ChainedMessagesProduct, std::optional<std::pair<uint16_t, uint16_t>>> store(binary_ptr packet, long long time_ms);

		static const long long defaultWaitTime = 250;
	};

	const shared_ptr<OutgoingStorage> outgoingStorage;
	const shared_ptr<IncomingStorage> incomingStorage;

public:
	RtcpNackResponder(unsigned maxStoredPacketCount = OutgoingStorage::defaultMaximumSize, long long maxWaitTime = IncomingStorage::defaultWaitTime);

	/// Checks for RTCP NACK and handles it,
	/// @param message RTCP message
	/// @returns unchanged RTCP message and requested RTP packets
	ChainedIncomingControlProduct processIncomingControlMessage(message_ptr message) override;

	/// Stores RTP packets in internal storage
	/// @param messages RTP packets
	/// @param control RTCP
	/// @returns Unchanged RTP and RTCP
	ChainedOutgoingProduct processOutgoingBinaryMessage(ChainedMessagesProduct messages,
														message_ptr control) override;

	ChainedIncomingProduct processIncomingBinaryMessage(ChainedMessagesProduct messages) override;
};

} // namespace rtc

#endif /* RTC_ENABLE_MEDIA */

#endif /* RTC_RTCP_NACK_RESPONDER_H */
