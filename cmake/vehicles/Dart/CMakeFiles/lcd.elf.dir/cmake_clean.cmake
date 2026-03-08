file(REMOVE_RECURSE
  "lcd.elf"
  "lcd.elf.pdb"
)

# Per-language clean rules from dependency scanning.
foreach(lang ASM C CXX)
  include(CMakeFiles/lcd.elf.dir/cmake_clean_${lang}.cmake OPTIONAL)
endforeach()
