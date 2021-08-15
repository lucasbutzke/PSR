"""
Autor: Lucas Cordeiro Butzke 

Codigo para simulacao de tarefas em regime RM e EDF,
verificar escalonabilidade e cumprimento de deadlines 
do processador.
"""
import sys
from simso.core import Model
from simso.configuration import Configuration


def mainRM():
    # Passar as tarefas atraves de um arquivo ou argumento adional
    configuration = Configuration()

    # Verificar tempo necessario de simulacao e quantas simulacoes realizar
    configuration.duration = 40 * configuration.cycles_per_ms

    # Adquirir tarefas do arquivo ou caso nao tiver por um default
    configuration.add_task(name="T1", identifier=1, period=2,
        activation_date=0, wcet=0.5, deadline=2)
    configuration.add_task(name="T2", identifier=2, period=3,
        activation_date=0, wcet=1.2, deadline=3)
    configuration.add_task(name="T3", identifier=3, period=6,
        activation_date=0, wcet=0.5, deadline=6)

    # Processadores do hardware de teste
    configuration.add_processor(name="CPU 1", identifier=1)

    # Realizar testes em RM e EDF, gerar um relatorio final das simulacoes
    #configuration.scheduler_info.filename = "examples/RM.py"
    configuration.scheduler_info.clas = "simso.schedulers.RM"

    # Check the config before trying to run it.
    configuration.check_all()

    # Init a model from the configuration.
    model = Model(configuration)

    # Execute the simulation.
    model.run_model()

    # Print logs.
    #for log in model.logs:
    #    print(log)

    """
    1 - relatorio da utilizacao total do processador
    2 - tempo de resposta minimo/medio/maximo de todas as tarefas
    3 - alguma tarefa perdeu seu deadline? Qual tarefa? Em que momento?
    """
    # Pegar ciclo maior (from functools import reduce (em Configuration.py))
    print(f"ciclo maior é de {configuration.get_hyperperiod()} ms")

    # Utilizacao total do processador    
    utilizacao = model.results.calc_load() # carrega um generator com todas as trocas de contexto e informacao de overhead
    u_total = next(utilizacao) # finalizo a geracao de tupla numa variavel
    print(f"taxa de utilização total é: {u_total[1] * 100}%")

    # pegar o tempo minimo/maximo/medio
    # separar tempo de inicio e termino por tarefa, depois pegar minimo/maximo/medio
    # Calculo U diferente de Urm
    valor_tempo = []
    for task in model.results.tasks:
        print("verificacao deadline" + task.name + ":")
        for job in task.jobs:
            # print("%s %.3f ms" % (job.name, job.computation_time))
            if job.is_active() == False:
                valor_tempo.append(job.actual_computation_time)
                if job.exceeded_deadline == True:
                    print("Job %s  pedeu deadline %.3f ms" % (job.name, job.computation_time))
    print(f"O menor valor de tempo de computação das tarefas é: {min(valor_tempo)}")
    print(f"O maior valor de tempo de computação das tarefas é: {max(valor_tempo)}")
    print(f"O valor médio de tempo de computação das tarefas é: {sum(valor_tempo)/len(valor_tempo)}")

    # Verificar cumprimento de deadlines
    """
    for task in model.results.tasks:
        print(task.name + ":")
        for job in task.jobs:
            if job.is_active() == False:    # job.abortaded or job.end_date # para outra verificacao
                try:    # para certeza de print da variavel
                    print(job.exceeded_deadline)
                except: # caso simulacao terminou antes de acabar a task
                    break
    """

def mainEDF():
    # Passar as tarefas atraves de um arquivo ou argumento adional
    configuration = Configuration()

    # Verificar tempo necessario de simulacao e quantas simulacoes realizar
    configuration.duration = 40 * configuration.cycles_per_ms

    # Adquirir tarefas do arquivo ou caso nao tiver por um default
    configuration.add_task(name="T1", identifier=1, period=2,
        activation_date=0, wcet=0.5, deadline=1.9)
    configuration.add_task(name="T2", identifier=2, period=5,
        activation_date=0, wcet=2, deadline=5)
    configuration.add_task(name="T3", identifier=3, period=1,
        activation_date=0, wcet=0.1, deadline=0.5)
    configuration.add_task(name="T4", identifier=4, period=10,
        activation_date=0, wcet=5, deadline=10)

    # Processadores do hardware de teste
    configuration.add_processor(name="CPU 1", identifier=1)

    # Realizar testes em RM e EDF, gerar um relatorio final das simulacoes
    #configuration.scheduler_info.filename = "examples/RM.py"
    configuration.scheduler_info.clas = "simso.schedulers.EDF"

    # Check the config before trying to run it.
    configuration.check_all()

    # Init a model from the configuration.
    model = Model(configuration)

    # Execute the simulation.
    model.run_model()

    # Print logs.
    #for log in model.logs:
    #    print(log)

    """
    1 - relatorio da utilizacao total do processador
    2 - tempo de resposta minimo/medio/maximo de todas as tarefas
    3 - alguma tarefa perdeu seu deadline? Qual tarefa? Em que momento?
    """
    # Pegar ciclo maior (from functools import reduce (em Configuration.py))
    print(f"ciclo maior é de {configuration.get_hyperperiod()} ms")

    # Utilizacao total do processador    
    utilizacao = model.results.calc_load() # carrega um generator com todas as trocas de contexto e informacao de overhead
    u_total = next(utilizacao) # finalizo a geracao de tupla numa variavel
    print(f"taxa de utilização total é: {u_total[1] * 100}%")

    # pegar o tempo minimo/maximo/medio
    valor_tempo = []
    for task in model.results.tasks:
        print("verificacao deadline " + task.name + " : ")
        for job in task.jobs:
            # print("%s %.3f ms" % (job.name, job.computation_time))
            if job.is_active() == False:
                valor_tempo.append(job.computation_time)
                if job.exceeded_deadline == True:
                    print("Job %s  pedeu deadline %.3f ms" % (job.name, job.computation_time))
    print(f"O menor valor de tempo de computação das tarefas é: {min(valor_tempo)}")
    print(f"O maior valor de tempo de computação das tarefas é: {max(valor_tempo)}")
    print(f"O valor médio de tempo de computação das tarefas é: {sum(valor_tempo)/len(valor_tempo)}")

    # Verificar cumprimento de deadlines
    """
    for task in model.results.tasks:
        print(task.name + ":")
        for job in task.jobs:
            if job.is_active() == False:    # job.abortaded or job.end_date # para outra verificacao
                try:    # para certeza de print da variavel
                    print(job.exceeded_deadline)
                except: # caso simulacao terminou antes de acabar a task
                    break
    """

# simulacao Rate Monotonic
# realizar leitura de arquivo com todas tarefas desejadas simular
mainRM()

# simulacao Earliest Deadline First
mainEDF()
