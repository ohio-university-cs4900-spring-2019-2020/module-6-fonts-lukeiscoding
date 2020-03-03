#pragma once

#include "NetMsg.h"
#include "Vector.h"
#include "Mat4.h"

namespace Aftr {
	class NetMsgSimpleWO : public NetMsg {
	public:
		NetMsgMacroDeclaration(NetMsgSimpleWO);

		virtual bool toStream(NetMessengerStreamBuffer& os) const;
		virtual bool fromStream(NetMessengerStreamBuffer& is);
		virtual void onMessageArrived();
		virtual std::string toString() const;

		Vector pos;
		Mat4 dma;
		int id;
		int new_indicator;
	protected:

	};
}// namespace Aftr