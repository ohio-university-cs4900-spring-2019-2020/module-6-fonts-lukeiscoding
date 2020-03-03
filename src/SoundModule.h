#pragma once

#include "irrKlang.h"
#include "Vector.h"
using namespace std;


namespace Aftr {
	class SoundManager {
	public:
		static void init();
		static irrklang::vec3df converter(const Vector & position);
		static void playBackGround(const char* soundFileName, bool playLooped, bool startPaused, bool track);
		static void play3DSound(const char* soundFileName, Vector position, bool playLooped, bool startPaused, bool track);
		static void changeEffect(); // it will crash
		static void setCamPosition(const Vector& pos, const Vector& lookdir, const Vector& vel, const Vector& updir);
		static vector<irrklang::ISound*> sound2D; //2D sound list
		static vector<irrklang::ISound*> sound3D; //3D sound list
	private:
		static irrklang::ISoundEngine* engine;
		static int sound_effct_number;
	};
}