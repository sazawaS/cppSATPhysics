#include <vector>
#include <algorithm>
#include <SFML/Graphics.hpp>

class Input {
public:
    std::vector<char*> currentInputList = {};
    std::vector<char*> prevFrameInputList = {};

    void updateInput()
    {
        prevFrameInputList.clear();
        prevFrameInputList = currentInputList;
        currentInputList.clear();
        if (sf::Keyboard::isKeyPressed(sf::Keyboard::Key::W)) currentInputList.push_back("W");
        if (sf::Keyboard::isKeyPressed(sf::Keyboard::Key::A)) currentInputList.push_back("A");
        if (sf::Keyboard::isKeyPressed(sf::Keyboard::Key::S)) currentInputList.push_back("S");
        if (sf::Keyboard::isKeyPressed(sf::Keyboard::Key::D)) currentInputList.push_back("D");
        
        if (sf::Mouse::isButtonPressed(sf::Mouse::Button::Left)) currentInputList.push_back("LMB");
        if (sf::Mouse::isButtonPressed(sf::Mouse::Button::Right)) currentInputList.push_back("RMB");
    }

    bool isKeyPressed(char* key)
    {
        if ( std::find(currentInputList.begin(), currentInputList.end(), key) != currentInputList.end())
        {
            return true;
        }
        return false;
    }

    bool isKeyJustPressed(char* key)
    {
        if (!isKeyPressed(key)) return false;
        if ( std::find(prevFrameInputList.begin(), prevFrameInputList.end(), key) == prevFrameInputList.end())
        {
            return true;
        }
        return false;
    }

    bool isKeyJustReleased(char* key)
    {
        if (isKeyPressed(key)) return false;
        if ( std::find(prevFrameInputList.begin(), prevFrameInputList.end(), key) != prevFrameInputList.end())
        {
            return true;
        }
        return false;
    }
};