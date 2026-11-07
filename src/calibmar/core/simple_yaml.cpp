#include "calibmar/core/simple_yaml.h"

#include <algorithm>
#include <cctype>
#include <fstream>
#include <sstream>
#include <stdexcept>

namespace {
  std::string Trim(const std::string& s) {
    auto begin = std::find_if_not(s.begin(), s.end(), [](unsigned char c) { return std::isspace(c); });
    auto end = std::find_if_not(s.rbegin(), s.rend(), [](unsigned char c) { return std::isspace(c); }).base();
    return begin < end ? std::string(begin, end) : std::string();
  }

  std::string StripComment(const std::string& line) {
    char quote = 0;
    for (size_t i = 0; i < line.size(); i++) {
      char c = line[i];
      if (quote) {
        if (c == quote) quote = 0;
      }
      else if (c == '\'' || c == '"') {
        quote = c;
      }
      else if (c == '#' && (i == 0 || std::isspace(static_cast<unsigned char>(line[i - 1])))) {
        return line.substr(0, i);
      }
    }
    return line;
  }

  std::string Unquote(const std::string& s) {
    if (s.size() >= 2 && (s.front() == '\'' || s.front() == '"') && s.back() == s.front()) {
      return s.substr(1, s.size() - 2);
    }
    return s;
  }

}

namespace calibmar {
  namespace simple_yaml {
    Values Parse(const std::string& content) {
      Values values;
      std::vector<std::pair<size_t, std::string>> parents;
      std::istringstream stream(content);
      std::string raw, pending_key;
      while (std::getline(stream, raw)) {
        std::string line = StripComment(raw);
        std::string trimmed = Trim(line);
        if (trimmed.empty()) {
          continue;
        }
        if (!pending_key.empty()) {
          values[pending_key] += " " + trimmed;
          if (trimmed.find(']') != std::string::npos) {
            pending_key.clear();
          }
          continue;
        }
        if (trimmed[0] == '%' || trimmed.rfind("---", 0) == 0 || trimmed[0] == '-') {
          continue;
        }

        size_t indent = line.find_first_not_of(" \t");
        size_t colon = trimmed.find(':');
        if (colon == std::string::npos) {
          continue;
        }
        while (!parents.empty() && parents.back().first >= indent) {
          parents.pop_back();
        }

        std::string key = Unquote(Trim(trimmed.substr(0, colon)));
        std::string value = Trim(trimmed.substr(colon + 1));
        if (value.rfind("!!", 0) == 0) {
          size_t space = value.find_first_of(" \t");
          value = space == std::string::npos ? "" : Trim(value.substr(space));
        }

        std::string full_key;
        for (const auto& parent : parents) {
          full_key += parent.second + ".";
        }
        full_key += key;

        if (value.empty()) {
          parents.push_back({indent, key});
          continue;
        }
        values[full_key] = value;
        if (value[0] == '[' && value.find(']') == std::string::npos) {
          pending_key = full_key;
        }
      }
      return values;
    }

    bool Has(const Values& values, const std::string& key) {
      return values.count(key) > 0;
    }

    std::string Text(const Values& values, const std::string& key, const std::string& fallback) {
      auto it = values.find(key);
      return it == values.end() ? fallback : Unquote(it->second);
    }

    double Number(const Values& values, const std::string& key) {
      std::string text = Text(values, key);
      std::istringstream stream(text);
      double value;
      if (text.empty() || !(stream >> value) || !(stream >> std::ws).eof()) {
        throw std::runtime_error("Invalid or missing number '" + key + "'");
      }
      return value;
    }

    std::optional<double> OptionalNumber(const Values& values, const std::string& key) {
      if (!Has(values, key)) {
        return {};
      }
      return Number(values, key);
    }

    std::vector<double> List(const Values& values, const std::string& key) {
      auto it = values.find(key);
      if (it == values.end()) {
        throw std::runtime_error("Missing list '" + key + "'");
      }
      std::string text = it->second;
      size_t open = text.find('['), close = text.rfind(']');
      if (open == std::string::npos || close == std::string::npos || close < open) {
        throw std::runtime_error("Invalid list '" + key + "'");
      }
      text = text.substr(open + 1, close - open - 1);
      std::replace(text.begin(), text.end(), ',', ' ');
      std::istringstream stream(text);
      std::vector<double> list;
      double v;
      while (stream >> v) {
        list.push_back(v);
      }
      if (!(stream >> std::ws).eof()) {
        throw std::runtime_error("Non-numeric values in list '" + key + "'");
      }
      return list;
    }

    std::optional<std::string> FirstKey(const Values& values, const std::vector<std::string>& keys) {
      for (const auto& key : keys) {
        if (Has(values, key)) {
          return key;
        }
      }
      return {};
    }

    std::string ReadFile(const std::string& path) {
      std::ifstream file(path);
      if (!file.is_open()) {
        throw std::runtime_error("Could not open " + path);
      }
      std::stringstream buffer;
      buffer << file.rdbuf();
      return buffer.str();
    }
  }
}
