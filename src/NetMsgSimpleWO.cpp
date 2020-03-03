#include "NetMsgSimpleWO.h"
#include <sstream>
#include <iostream>
#include "GLViewNewModule.h"
#include "ManagerGLView.h"
#include "WorldContainer.h"
#include "Model.h"

using namespace Aftr;

NetMsgMacroDefinition(NetMsgSimpleWO);

bool NetMsgSimpleWO::toStream(NetMessengerStreamBuffer & os) const
{
	os << pos.x << pos.y << pos.z << id << new_indicator << dma[0] << dma[1] << dma[2] << dma[3] << dma[4] << dma[5] << dma[6] << dma[7] << dma[8] << dma[9] << dma[10] << dma[11] << dma[12] << dma[13] << dma[14] << dma[15];
	return true;
}

bool NetMsgSimpleWO::fromStream(NetMessengerStreamBuffer & is)
{
	is >> pos.x >> pos.y >> pos.z >> id >> new_indicator >> dma[0] >> dma[1] >> dma[2] >> dma[3] >> dma[4] >> dma[5] >> dma[6] >> dma[7] >> dma[8] >> dma[9] >> dma[10] >> dma[11] >> dma[12] >> dma[13] >> dma[14] >> dma[15];
	return true;
}

void NetMsgSimpleWO::onMessageArrived()
{
	if (new_indicator == 0) {
		WO* wo = ((GLViewNewModule*)ManagerGLView::getGLView())->getActorLst()->at(id);
		wo->getModel()->setDisplayMatrix(dma);
		wo->setPosition(pos);
	}
	else {
		std::string jet(ManagerEnvironmentConfiguration::getSMM() + "/models/jet_wheels_down_PP.wrl");
		WO* wo1 = WO::New(jet, Vector(1, 1, 1), MESH_SHADING_TYPE::mstFLAT);
		wo1->setPosition(Vector(10, 0, 50));
		wo1->renderOrderType = RENDER_ORDER_TYPE::roOPAQUE;
		wo1->setLabel("jetnew");
		((GLViewNewModule*)ManagerGLView::getGLView())->getWorldContainer()->push_back(wo1);
		((GLViewNewModule*)ManagerGLView::getGLView())->getActorLst()->push_back(wo1);
	}
}

std::string NetMsgSimpleWO::toString() const
{
	std::stringstream ss;
	ss << NetMsg::toString();
	ss << "Payload: \n"
		<< "p:{" << pos.x << "," << pos.y << "," << pos.z << "} id:" << id << ",new_indicator:" << new_indicator << ",Matrix:" << dma[0] << dma[1] << dma[2] << dma[3] << dma[4] << dma[5] << dma[6] << dma[7] << dma[8] << dma[9] << dma[10] << dma[11] << dma[12] << dma[13] << dma[14] << dma[15] << "\n";
	return ss.str();
}
