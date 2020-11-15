//
// Created by luka on 18. 09. 20.
//

#include <chrono>
#include <iostream>
#include "LetterCubes.h"
#include "ARElements.h"

LetterCubes::LetterCubes(const std::string &text, float gridSize, float duration, float delay) : text(text), letter_dur(duration), delay(delay), gridSize(gridSize) {
    currentLetter = -1;
}

void LetterCubes::Show() {
    if (currentLetter < 0) {
        currentLetter = 0;
        start = std::chrono::steady_clock::now();
    }


    double elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - start).count();
    if (elapsed > delay + letter_dur) {
        currentLetter++;
        if (currentLetter > text.size()-1)
            currentLetter = 0;
        start = std::chrono::steady_clock::now();
        elapsed = 0;
    }

    DrawLetter(text.at(currentLetter), (elapsed-delay)/letter_dur, elapsed >= delay);
}

void LetterCubes::DrawLetter(const char letter, double progress, bool show) {
    float d = gridSize/10;
    float offset = d/2;
    for (int i = 0; i < supportedLetters.size(); i++) {
        if (supportedLetters[i] == letter) {
            auto def = letterDef[i];

            int idx = 0;
            for (int r = 0; r < 5; r++) {
                for (int c = 0; c < 5; c++) {
                    // Show cubes needed for current letter
                    if (show && def.find(idx) != def.end()) {
                        double depth;
                        if (progress > 0.5f) {
                            depth = 2*d - (progress-0.5f) * 4*d;
                        } else {
                            depth = progress * 4*d;
                        }
                        ARElements::DrawCube(offset+c*2*d, offset+r*2*d, depth/2.0, d, d, depth, 0, 1, 0, 0.001);
                    } else {
                        ARElements::DrawCube(offset+c*2*d, offset+r*2*d, 0, d, d, 0.001, 0.2, 0.2, 0.2, 0.001);
                    }
                    idx++;
                }
            }
        }
    }

}
