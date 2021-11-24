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

#ifdef _WIN32
#include <winsock.h>
#else
#include <arpa/inet.h>
#endif

#include <cstring>

#include "mediahandlerrootelement.hpp"

namespace rtc {

message_ptr MediaHandlerRootElement::reduce(ChainedMessagesProduct messages) {
	if (messages && !messages->empty()) {
		std::vector<binary_ptr> filtered_messages{};
		filtered_messages.reserve(messages->size());
		size_t total_size = 1;
		for (auto message: *messages) {
			if (message && !message->empty()) {
				total_size += 4 + message->size();
				filtered_messages.push_back(message);
			}
		}
		if (filtered_messages.empty()) {
			return nullptr;
		}
        const auto msgs = make_message(total_size);
		auto data = reinterpret_cast<uint8_t *>(msgs->data());
		*data = 0;
		data = data + 1;
		for (auto message: filtered_messages) {
			auto size_ptr = reinterpret_cast<uint32_t *>(data);
			*size_ptr = htonl(message->size());
			memcpy(data + 4, message->data(), message->size());
			data = data + 4 + message->size();
		}
		return msgs;
	} else {
		return nullptr;
	}
}

ChainedMessagesProduct MediaHandlerRootElement::split(message_ptr message) {
	return make_chained_messages_product(message);
}

} // namespace rtc

#endif /* RTC_ENABLE_MEDIA */
