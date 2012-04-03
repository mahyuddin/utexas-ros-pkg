#include <ar_nav_common/PatternParser.h>

using namespace ar_nav;

int main(int argc, const char *argv[]) {

  if (argc != 3) {
    std::cerr << "USAGE: generate_ar_file file.yaml ar_pattern_file\n";
    return 1;
  }

  std::string inFile(argv[1]);
  std::string outFile(argv[2]);

  std::vector<Pattern> patterns;
  readPatternFile(inFile, patterns);
  writeARPatternFile(outFile, patterns);
  
  return 0;
}
