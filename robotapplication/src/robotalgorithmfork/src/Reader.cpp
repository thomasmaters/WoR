#include <readline/readline.h>
#include <readline/history.h>
#include <boost/algorithm/string.hpp>
#include <boost/algorithm/string/trim.hpp>
#include <fstream>
#include "Reader.hpp"
#include "Specification.hpp"

using namespace std;

vector<Specification> Reader::readBatchfile(const string& filename) {
    vector<Specification> specs;
    Specification spec;
    ifstream file(filename);

    if (!file.is_open()) {
        throw runtime_error("cannot open " + filename);
    }

    string line;
    while (readUserInput(file, line)) {

        if (spec.strToKnownSpecs(line)) {
            specs.push_back(spec);
        }

    }
    return specs;
}

bool Reader::writeBatchOutput(const string& filename, const string& data) {
    ofstream file;
    file.open(filename);
    
    if(!file.is_open()) {
        return false;
    }
    file << data;
    file.close();
    return true;
}

bool Reader::readUserInput(istream& is, string& line) {
    size_t pos = 0;
    if (&is == &cin) {
        line = readline("\033[1;32mZoek: \033[0m");
        add_history(line.c_str());
    } else {
        getline(is, line);
    }

    if ((pos = line.find_first_of("#")) != string::npos) {
        line.erase(pos);
    }
    boost::algorithm::trim_left(line);
    boost::algorithm::trim_right(line);
    boost::to_lower(line);

    return !line.empty();
}
