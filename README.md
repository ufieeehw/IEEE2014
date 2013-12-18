IEEE2014
========


#### Errata

In order to use floating point numbers in printf/stdio functions, in Atmel Studio 6.1, press Alt-F7 to open the project properties. 

Under the toolchain tab, click on `AVR/GNU Linker > General` and select the box labeled `Use vprintf library`. 

Next, click on `AVR/GNU Linker > Libraries` and add the library titled `printf_flt` by clicking on the green `Add Item` button.

Th-Th-Th-That's all, folks!
