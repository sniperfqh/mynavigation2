#include "nav2_regulated_modules/detail/terminal_position.hpp"

int main()
{
  using nav2_regulated_modules::detail::terminalPosition;
  const auto crossing = terminalPosition(0.0075, 0.01, 0.005, true, false, false);
  if (!crossing.stopped || !crossing.accurate)
  {
    return 1;
  }
  const auto outside = terminalPosition(0.012, 0.01, 0.005, true, true, true);
  if (!outside.stopped || outside.accurate)
  {
    return 2;
  }
  const auto approaching = terminalPosition(0.009, 0.01, 0.005, false, false, false);
  if (approaching.stopped || approaching.accurate)
  {
    return 3;
  }
  const auto entry = terminalPosition(0.004, 0.01, 0.005, false, false, false);
  if (!entry.stopped || !entry.accurate)
  {
    return 4;
  }
  const auto legacy_entry = terminalPosition(0.009, 0.01, 0.0, false, false, false);
  const auto legacy_latch = terminalPosition(0.012, 0.01, 0.0, false, true, true);
  if (!legacy_entry.stopped || !legacy_entry.accurate || !legacy_latch.stopped || !legacy_latch.accurate)
  {
    return 5;
  }
  return 0;
}
