#include <atomic>
#include <string>
#include <thread>

class AsyncGetline
{
  public:
    AsyncGetline()
    {
        input = "";
        sendOverNextLine = true;
        continueGettingInput = true;
        std::thread([&]() {
            std::string synchronousInput;
            char nextCharacter;

            do
            {
                synchronousInput = "";

                while (continueGettingInput)
                {
                    while (std::cin.peek() == EOF)
                    {
                        std::this_thread::yield();
                    }

                    nextCharacter = std::cin.get();

                    if (nextCharacter == '\n')
                    {
                        break;
                    }

                    synchronousInput += nextCharacter;
                }

                if (!continueGettingInput)
                {
                    break;
                }

                while (continueGettingInput && !sendOverNextLine)
                {
                    std::this_thread::yield();
                }

                if (!continueGettingInput)
                {
                    break;
                }

                inputLock.lock();
                input = synchronousInput;
                inputLock.unlock();

                sendOverNextLine = false;
            } while (continueGettingInput && input != "exit");
        }).detach();
    }

    ~AsyncGetline()
    {
        continueGettingInput = false;
    }

    std::string GetLine()
    {
        if (sendOverNextLine)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            return "";
        }
        else
        {
            inputLock.lock();
            std::string returnInput = input;
            inputLock.unlock();

            sendOverNextLine = true;

            return returnInput;
        }
    }

  private:
    std::atomic<bool> continueGettingInput;
    std::atomic<bool> sendOverNextLine;
    std::mutex inputLock;
    std::string input;
};
