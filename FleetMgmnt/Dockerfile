FROM python:3

WORKDIR /usr/src/app

COPY . .
RUN pip install --no-cache-dir -r requirements.txt
RUN rm -rd maps
RUN rm -rd misc
RUN rm config.json

CMD [ "python", "./main.py" ]
