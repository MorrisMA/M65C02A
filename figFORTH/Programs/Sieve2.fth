8192 constant size
0 variable flags size 1 - allot
: sieve
    flags size 1+ 1 fill
    0 size 0
    do flags i + c@
        if i dup + 3 + dup i +
            begin dup size <
            while 0 over flags + c! over + repeat
            drop drop 1+
        then
    loop ;
: primes ." S" 10 1 do sieve sp! loop sp! ." E" ;
 