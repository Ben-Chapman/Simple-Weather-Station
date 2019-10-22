import json
import os

from Adafruit_IO import Client, Data, Feed
import requests

def get_weather(latitude, longitude):
    darksky_api_url = '  https://api.darksky.net/forecast/'
    darksky_api_key = os.environ['DARKSKY_API_KEY']

    request_params = {
        'exclude': 'currently,flags, daily',
        'units': 'uk2',
    }

    r = requests.get(darksky_api_url + '{key}/{latitude},{longitude}'.format(
        key=darksky_api_key,
        latitude=latitude,
        longitude=longitude
    ),
    params=request_params)

    response_data = json.loads(r.text)

    weather_emoji = {
        'default': '🤔',
        'clear-day': '🌞',
        'clear-night': '🌙',
        'rain': '☔ ',
        'snow': '❄️ ',
        'sleet': '🌨️ ',
        'wind': '🌬️ ',
        'fog': '🌫️ ',
        'cloudy': '☁️' ,
        'partly-cloudy-day': '🌤️ ',
        'partly-cloudy-night': '🌥️ ',
    }

    forecast_data = []

    # Build the forecast string with the weather emoji
    for f in 'minutely', 'hourly':
        summary = response_data[f]['summary'].replace('.', '')

        if response_data[f]['icon'] in weather_emoji:
            emoji = weather_emoji[response_data[f]['icon']]
        else:
            emoji = weather_emoji['default']

        icon = response_data[f]['icon']
        forecast = '{emoji} {summary}'.format(
            emoji=emoji,
            summary=summary)

        forecast_data.append(forecast)

    return forecast_data


def update_adafruitio(weather_forecast):
    aio_username = os.environ['AIO_USERNAME']
    aio_api_key = os.environ['AIO_API_KEY']
    aio_feeds = [
        'weather-station.darksky-minutely',
        'weather-station.darksky-hourly'
    ]
    
    # Setting up the aio connection
    aio = Client(aio_username, aio_api_key)

    # Retrieving a list of all forecast data values and deleting them as we don't need that data to persist
    for feed in aio_feeds:
        d = aio.data(feed)
        for data in d:
            aio.delete(feed, data.id)
        
    # Now send the forecasts to Adafruit
    for i in range(0,2):  # 0 == Minutely, 1 == Hourly
        aio.send_data(aio_feeds[i], weather_forecast[i])

w = get_weather(latitude='40.7369597',longitude='-74.3029359')

update_adafruitio(w)