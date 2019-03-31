def check_prime_number(numbers):
    pnres = []
    for num in numbers:
        pn = True
        for i in range(2, num):
            if num % i == 0:
                pn = False
                print("check_prime_number: {} / {} = {}".format(num, i, num//i))
                break
        pnres.append(pn)
    return pnres
