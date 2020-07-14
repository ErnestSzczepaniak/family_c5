# ---------------------------------------------| config |--------------------------------------------- #

set(platform                    target)

set(type                        debug)

set(version_major               1)
set(version_minor               0)
set(version_revision            0)

option(dump ""                  on)
option(image ""                 on)
option(documentation ""         off)

# ---------------------------------------------| project |--------------------------------------------- #

set(name_library                family_c5)

set(name_executable             executable.elf)

set(dependency_internal_common  )
set(dependency_internal_target  )
set(dependency_internal_host    )

set(dependency_external_common  )
set(dependency_external_target  )
set(dependency_external_host    )

set(dependency_scope            PRIVATE)

# ---------------------------------------------| target |--------------------------------------------- #

set(architecture                v7a)
set(core                        a9)
set(gic                         pl390)
set(family                      c5)

set(board                       de10standard)

set(memory                      dual)

set(sdram_start                 0x00000000)
set(sdram_size                  1024M)
set(sdram_size_stack            2M)
set(sdram_size_pool             512M)

set(ocram_start                 0xffff0000)
set(ocram_size                  64K)
set(ocram_size_stack            4K)
set(ocram_size_pool             48K)

