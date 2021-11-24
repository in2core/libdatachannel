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

#include "genericmediahandlerelement.hpp"

namespace rtc {

GenericMediaHandlerElement::GenericMediaHandlerElement(): MediaHandlerElement() { };

ChainedIncomingControlProduct GenericMediaHandlerElement::processIncomingControlMessage(message_ptr messages) {
	if (this->_processIncomingControlMessage) {
		return this->_processIncomingControlMessage.value()(messages);
	}
	return MediaHandlerElement::processIncomingControlMessage(messages);
}

message_ptr GenericMediaHandlerElement::processOutgoingControlMessage(message_ptr messages) {
	if(this->_processOutgoingControlMessage) {
		return this->_processOutgoingControlMessage.value()(messages);
	}
	return MediaHandlerElement::processOutgoingControlMessage(messages);
}

ChainedIncomingProduct GenericMediaHandlerElement::processIncomingBinaryMessage(ChainedMessagesProduct messages) {
	if(this->_processIncomingBinaryMessage) {
		return this->_processIncomingBinaryMessage.value()(messages);
	}
	return MediaHandlerElement::processIncomingBinaryMessage(messages);
}

ChainedOutgoingProduct GenericMediaHandlerElement::processOutgoingBinaryMessage(ChainedMessagesProduct messages, message_ptr control) {
	if(this->_processOutgoingBinaryMessage) {
		return this->_processOutgoingBinaryMessage.value()(messages, control);
	}
	return MediaHandlerElement::processOutgoingBinaryMessage(messages, control);
}

GenericMediaHandlerElement* GenericMediaHandlerElement::onProcessIncomingControlMessage(std::function<ChainedIncomingControlProduct (message_ptr)> _processIncomingControlMessage) {
	this->_processIncomingControlMessage = _processIncomingControlMessage;
	return this;
}

GenericMediaHandlerElement* GenericMediaHandlerElement::onProcessOutgoingControlMessage(std::function<message_ptr (message_ptr)> _processOutgoingControlMessage) {
	this->_processOutgoingControlMessage = _processOutgoingControlMessage;
	return this;
}

GenericMediaHandlerElement* GenericMediaHandlerElement::onProcessIncomingBinaryMessage(std::function<ChainedIncomingProduct (ChainedMessagesProduct)> _processIncomingBinaryMessage) {
	this->_processIncomingBinaryMessage = _processIncomingBinaryMessage;
	return this;
}

GenericMediaHandlerElement* GenericMediaHandlerElement::onProcessOutgoingBinaryMessage(std::function<ChainedOutgoingProduct (ChainedMessagesProduct, message_ptr)> _processOutgoingBinaryMessage) {
	this->_processOutgoingBinaryMessage = _processOutgoingBinaryMessage;
	return this;
}

} // namespace rtc

#endif /* RTC_ENABLE_MEDIA */
