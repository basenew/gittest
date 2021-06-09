#pragma once

#include <string>
#include <cstdio>
#include <iostream>
#include <ctime>

namespace sched
{
	typedef enum
	{
		YEAR,
		MONTH,
		DAY,
		HOUR,
		MINUTE,
		SECOND,
		WEEK
	} TimeUnit;

	typedef enum
	{
		YEARLY,
		MONTHLY,
		DAYLY,
		HOURLY,
		MINUTELY,
		SECONDLY,
		WEEKLY
	} FreqUnit;

	typedef enum
	{
		MO = 1,
		TU,
		WE,
		TH,
		FR,
		SA,
		SU
	} WeekUnit;

	typedef enum
	{
		Jan = 1,
		Feb = 2,
		Mar = 3,
		Apr = 4,
		May = 5,
		Jun = 6,
		Jul = 7,
		Aug = 8,
		Sep = 9,
		Oct = 10,
		Nov = 11,
		Dec = 12
	} MonthUnit;

	class Date
	{
	private:
		const char Compare(const Date &OtherDate, TimeUnit FromPart = YEAR) const
		{
			for (; FromPart <= SECOND; FromPart = (TimeUnit)(FromPart + 1))
			{
				if (Data[FromPart] < OtherDate.Data[FromPart])
					return -1;
				else if (Data[FromPart] > OtherDate.Data[FromPart])
					return 1;
			}
			return 0;
		};

		short Data[6];
		class DatePart;

	public:
		Date()
		{
			Clear();
		}

		short DateInYear() const
		{
			return Data[YEAR];
		}

		short DateInMonth() const
		{
			return Data[MONTH];
		}

		short DateInDay() const
		{
			return Data[DAY];
		}

		short DateOfHour() const
		{
			return Data[HOUR];
		}

		short DateOfMinute() const
		{
			return Data[MINUTE];
		}

		short DateOfSecond() const
		{
			return Data[SECOND];
		}
		WeekUnit GetWeekDay()
		{ //retuns the day number
			int t[] = {0, 3, 2, 5, 0, 3, 5, 1, 4, 6, 2, 4};
			short yy = Data[YEAR], mm = Data[MONTH], dd = Data[DAY];
			yy -= mm < 3;
			switch ((yy + yy / 4 - yy / 100 + yy / 400 + t[mm - 1] + dd) % 7)
			{
			case 0:
				return SU;
			case 1:
				return MO;
			case 2:
				return TU;
			case 3:
				return WE;
			case 4:
				return TH;
			case 5:
				return FR;
			case 6:
				return SA;
			default:
				return MO;
			}
		}

		bool IsLeapYear(short Year) const
		{
			return ((Year % 4 == 0 && Year % 100 != 0) || Year % 400 == 0);
		}

		short DaysInMonth(short Month = 0) const
		{
			switch (Month == 0 ? Data[MONTH] : Month)
			{
			case 1:
				return 31;
			case 2:
				return IsLeapYear(Data[YEAR]) ? 29 : 28;
			case 3:
				return 31;
			case 4:
				return 30;
			case 5:
				return 31;
			case 6:
				return 30;
			case 7:
				return 31;
			case 8:
				return 31;
			case 9:
				return 30;
			case 10:
				return 31;
			case 11:
				return 30;
			case 12:
				return 31;
			}
			return 0;
		}

		std::string Format() const;
		operator std::string() const;
		Date &operator=(const std::string &Text);
		friend std::ostream &operator<<(std::ostream &stream, const Date &Date)
		{
			stream << Date.operator std::string();
			return stream;
		}
		bool operator<=(const Date &OtherDate) const { return (Compare(OtherDate) <= 0); }
		bool operator>=(const Date &OtherDate) const { return (Compare(OtherDate) >= 0); }
		bool operator<(const Date &OtherDate) const { return (Compare(OtherDate) < 0); }
		bool operator>(const Date &OtherDate) const { return (Compare(OtherDate) > 0); }
		bool operator==(const Date &OtherDate) const { return (Compare(OtherDate) == 0); }
		DatePart operator[](TimeUnit PartName);

		unsigned long Difference(Date &OtherDate, TimeUnit Unit, bool RoundUp = true) const;
		void SetToNow();
		bool IsEmpty() const { return (Data[YEAR] == 0 || Data[MONTH] == 0 || Data[DAY] == 0 ); }
		void Clear(bool OnlyTime = false)
		{
			if (!OnlyTime)
				Data[YEAR] = Data[MONTH] = Data[DAY] = 0;

			Data[HOUR] = Data[MINUTE] = Data[SECOND] = 0;
			WithTime = false;
		}

		void ConvertTimestampToDate(long timestamp);
		long ConvertDateToTimestamp();
		bool WithTime;
	};

	class Date::DatePart
	{
	private:
		Date &BaseDate;
		TimeUnit Part;

	public:
		DatePart(Date &BaseDate, TimeUnit Part) : BaseDate(BaseDate), Part(Part) {}

		operator short() const
		{
			if (Part <= SECOND)
				return BaseDate.Data[Part];
			else if (Part == WEEK)
			{
				// actually, this case is unused
				short Value = 0;
				for (char i = 1; i < BaseDate.Data[MONTH]; ++i)
					Value += BaseDate.DaysInMonth(i);
				Value += BaseDate.Data[DAY];

				return (Value - 1) / 7 + 1;
			}

			return 0;
		}

		DatePart &operator=(short Value)
		{
			if (Part <= SECOND)
				BaseDate.Data[Part] = Value;

			return *this;
		}

		DatePart &operator=(const DatePart &DatePart)
		{
			return operator=((short)DatePart);
		}

		DatePart &operator+=(short Value);
		DatePart &operator-=(short Value);
		DatePart &operator++()
		{
			operator+=(1);
			return *this;
		}
	};

	inline Date::DatePart Date::operator[](TimeUnit Part)
	{
		return DatePart(*this, Part);
	}

} // namespace sched