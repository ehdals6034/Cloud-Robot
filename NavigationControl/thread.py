from threading import Thread

def work(id, start, end, result):
    total = 0
    for i in range(start, end):
        total += i
    result.append(total)
    return result


if __name__ == "__main__":
    START, END = 0, 100000000
    result = list()
    th1 = Thread(target=work, args=(1, START, END, result))
    
    th1.start() #쓰레드 시작
    th1.join() #끝날때까지 기다림

print(f"Result: {sum(result)}")


