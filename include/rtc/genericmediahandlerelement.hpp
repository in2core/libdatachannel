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

#ifndef RTC_GENERIC_MEDIA_HANDLER_ELEMENT_H
#define RTC_GENERIC_MEDIA_HANDLER_ELEMENT_H

#if RTC_ENABLE_MEDIA

#include "mediahandlerelement.hpp"

namespace rtc {

class RTC_CPP_EXPORT GenericMediaHandlerElement final: public MediaHandlerElement {
	optional<std::function<ChainedIncomingControlProduct (message_ptr)>> _processIncomingControlMessage = nullopt;
	optional<std::function<message_ptr (message_ptr)>> _processOutgoingControlMessage = nullopt;
	optional<std::function<ChainedIncomingProduct (ChainedMessagesProduct)>> _processIncomingBinaryMessage = nullopt;
	optional<std::function<ChainedOutgoingProduct (ChainedMessagesProduct, message_ptr)>> _processOutgoingBinaryMessage = nullopt;
public:
	GenericMediaHandlerElement();

	ChainedIncomingControlProduct processIncomingControlMessage(message_ptr messages) override;
	message_ptr processOutgoingControlMessage(message_ptr messages) override;
	ChainedIncomingProduct processIncomingBinaryMessage(ChainedMessagesProduct messages) override;
	ChainedOutgoingProduct processOutgoingBinaryMessage(ChainedMessagesProduct messages, message_ptr control) override;

	GenericMediaHandlerElement* onProcessIncomingControlMessage(std::function<ChainedIncomingControlProduct (message_ptr)> _processIncomingControlMessage);
	GenericMediaHandlerElement* onProcessOutgoingControlMessage(std::function<message_ptr (message_ptr)> _processOutgoingControlMessage);
	GenericMediaHandlerElement* onProcessIncomingBinaryMessage(std::function<ChainedIncomingProduct (ChainedMessagesProduct)> _processIncomingBinaryMessage);
	GenericMediaHandlerElement* onProcessOutgoingBinaryMessage(std::function<ChainedOutgoingProduct (ChainedMessagesProduct, message_ptr)> _processOutgoingBinaryMessage);
};

} // namespace rtc

#endif // RTC_ENABLE_MEDIA

#endif // RTC_GENERIC_MEDIA_HANDLER_ELEMENT_H
