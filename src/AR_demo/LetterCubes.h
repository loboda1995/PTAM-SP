//
// Created by luka on 18. 09. 20.
//

#ifndef PTAM_LETTERCUBES_H
#define PTAM_LETTERCUBES_H


#include <string>
#include <set>
#include <vector>

class LetterCubes {
public:
    LetterCubes(const std::string &text, float gridSize, float duration, float delay);
    void Show();
private:
    float gridSize;
    float letter_dur;
    float delay;
    std::string text;

    std::chrono::time_point<std::chrono::steady_clock, std::chrono::duration<float>> start;
    int currentLetter = 0;
    int state = 0;

    void DrawLetter(const char letter, double progress, bool show);

    std::vector<char>supportedLetters {'L', 'U', 'K', 'A', 'E', 'N', 'P', 'T', 'M'};
    std::vector<std::set<int>>letterDef {
            {0, 1, 2, 3, 4, 5, 10, 15, 20},
            {1, 2, 3, 5, 9, 10, 14, 15, 19, 20, 24},
            {0, 3, 5, 7, 10, 11, 15, 17, 20, 23},
            {0, 4, 5, 9, 10, 11, 12, 13, 14, 15, 19, 21, 22, 23},
            {0, 1, 2, 3, 4, 5, 10, 11, 12, 13, 14, 15, 20, 21, 22, 23, 24},
            {0, 4, 5, 8, 9, 10, 12, 14, 15, 16, 19, 20, 24},
            {0, 5, 10, 11, 12, 13, 15, 18, 20, 21, 22, 23},
            {2, 7, 12, 17, 20, 21, 22, 23, 24},
            {0, 4, 5, 9, 10, 12, 14, 15, 16, 18, 19, 20, 24}
    };
};


#endif //PTAM_LETTERCUBES_H
