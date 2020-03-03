#include "SoundModule.h"

using namespace Aftr;

irrklang::ISoundEngine* SoundManager::engine;
vector<irrklang::ISound*> SoundManager::sound2D; //2D sound list
vector<irrklang::ISound*> SoundManager::sound3D; //3D sound list
int  SoundManager::sound_effct_number = 0;

// Initialize
void SoundManager::init() {
	// start the sound engine with default parameters
	engine = irrklang::createIrrKlangDevice();
	if (!engine)
	{
		printf("Could not startup engine\n"); // error starting up the engine
		return;
	}
}

// play background 2D sound
void Aftr::SoundManager::playBackGround(const char * soundFileName, bool playLooped, bool startPaused, bool track)
{
	if (!engine)
	{
		printf("No engine\n"); // no engine
		return;
	}
	sound2D.push_back(engine->play2D(soundFileName, playLooped, startPaused, track));
}

// play 3D sound
void Aftr::SoundManager::play3DSound(const char* soundFileName, Vector position, bool playLooped, bool startPaused, bool track)
{
	if (!engine)
	{
		printf("No engine\n"); // no engine
		return;
	}
	sound3D.push_back(engine->play3D(soundFileName, converter(position), playLooped, startPaused, track));
}

// change sound effct
// it will crash!!!
void Aftr::SoundManager::changeEffect()
{
	irrklang::ISoundEffectControl* fx = sound2D.at(0)->getSoundEffectControl();
	if (sound_effct_number == 0) {
		fx->disableAllEffects();
		fx->enableChorusSoundEffect();
		sound_effct_number = 1;
		return;
	};
	if (sound_effct_number == 1) {
		fx->disableAllEffects();
		fx->enableCompressorSoundEffect();
		sound_effct_number = 2;
		return;
	};
	if (sound_effct_number == 2) {
		fx->disableAllEffects();
		fx->enableDistortionSoundEffect();
		sound_effct_number = 3;
		return;
	};
	if (sound_effct_number == 3) {
		fx->disableAllEffects();
		fx->enableEchoSoundEffect();
		sound_effct_number = 4;
		return;
	};
	if (sound_effct_number == 4) {
		fx->disableAllEffects();
		fx->enableFlangerSoundEffect();
		sound_effct_number = 5;
		return;
	};
	if (sound_effct_number == 5) {
		fx->disableAllEffects();
		fx->enableGargleSoundEffect();
		sound_effct_number = 6;
		return;
	};
	if (sound_effct_number == 6) {
		fx->disableAllEffects();
		fx->enableI3DL2ReverbSoundEffect();
		sound_effct_number = 0;
		return;
	};
}

// set cam position
void Aftr::SoundManager::setCamPosition(const Vector & pos, const Vector & lookdir, const Vector & vel, const Vector & updir)
{
	engine->setListenerPosition(converter(pos), converter(lookdir), converter(vel), converter(updir)); //update the cam position
}

// this is a converter for the cam position
irrklang::vec3df SoundManager::converter(const Vector& position) {
	return irrklang::vec3df(position.x, -1 * position.y, position.z); // convert the vector system
}
